from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel
from typing import List, Optional, Union
from app.core.agent_builder import create_agent
from app.core.neon_service import neon_service
from sse_starlette.sse import EventSourceResponse
import asyncio
import json
import logging


# Set up logging
logger = logging.getLogger(__name__)


router = APIRouter()


class MessageRequest(BaseModel):
    message: str
    session_id: str
    context_text: Optional[str] = None


class Source(BaseModel):
    title: str
    section: str
    page: Union[int, str]


class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: List[Source]


@router.on_event("startup")
async def startup_event():
    """Initialize the Neon Postgres service when the app starts"""
    await neon_service.connect()


@router.on_event("shutdown")
async def shutdown_event():
    """Close the Neon Postgres service when the app shuts down"""
    await neon_service.close()


@router.post("/", response_model=ChatResponse)
async def chat(request: MessageRequest):
    """
    Send a message to the chatbot and receive a response.
    """
    try:
        logger.info(f"Processing chat request for session {request.session_id}")

        # Create or update the session
        await neon_service.create_session(request.session_id)

        # Create the agent for this session
        agent = create_agent(request.session_id)

        # Invoke the agent
        result = await agent.invoke(request.message, request.context_text)

        logger.info(f"Successfully processed chat request for session {request.session_id}")

        return ChatResponse(
            response=result["response"],
            session_id=result["session_id"],
            sources=result["sources"]
        )
    except Exception as e:
        logger.error(f"Error processing chat request for session {request.session_id}: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/stream")
async def chat_stream(request: Request, message_request: MessageRequest):
    """
    Send a message to the chatbot and receive a streamed response using Server-Sent Events.
    """
    async def event_generator():
        logger.info(f"Processing streaming chat request for session {message_request.session_id}")
        try:
            # Create or update the session
            await neon_service.create_session(message_request.session_id)

            # Create the agent for this session
            agent = create_agent(message_request.session_id)

            # Try to get a response from the agent
            try:
                # For streaming, we'll use the Groq client directly with the agent's context
                # Retrieve relevant context from the textbook with retry logic
                retrieval_results = agent.retriever.retrieve(message_request.message)

                # Get conversation history
                history = await agent.memory.get_session_history(message_request.session_id)

                # Format the history for the LLM
                chat_history = []
                for msg in history:
                    chat_history.append({
                        "role": msg.role,
                        "content": msg.content
                    })

                # Add the user's current message
                chat_history.append({
                    "role": "user",
                    "content": message_request.message
                })

                # Create system message with retrieved context
                system_message_content = (
                    "You are an intelligent tutor for the Physical AI & Humanoid Robotics textbook. "
                    "Answer questions based on the following context from the textbook. "
                    "If the context doesn't contain the information to answer the question, "
                    "acknowledge that and suggest the student look in the textbook for more information. "
                    "Always be clear, concise, and beginner-friendly in your explanations. "
                    "Reference specific textbook sections when possible.\n\n"
                    f"Context from textbook:\n"
                )

                # Add retrieved context to system message
                for i, result in enumerate(retrieval_results):
                    source = result.get('source_metadata', {})
                    page = source.get('page', 'unknown')
                    chapter = source.get('chapter', 'unknown')
                    system_message_content += f"\n--- Source {i+1}: Chapter {chapter}, Page {page} ---\n"
                    system_message_content += result['text_content'] + "\n"

                # Create the system message
                system_message = {
                    "role": "system",
                    "content": system_message_content
                }

                # Prepend system message to chat history
                chat_history.insert(0, system_message)

                # Call the LLM with streaming
                completion = agent.groq_client.chat.completions.create(
                    messages=chat_history,
                    model="llama-3.1-8b-instant",  # Using Groq's llama3.1 model
                    temperature=0.7,
                    max_tokens=1024,
                    stream=True  # Enable streaming
                )

                full_response = ""

                # Stream the response from the LLM
                for chunk in completion:
                    if chunk.choices[0].delta.content is not None:
                        token = chunk.choices[0].delta.content
                        full_response += token

                        # Send token as Server-Sent Event
                        yield {
                            "event": "message",
                            "data": json.dumps({"token": token})
                        }

                # Save the interaction to memory
                await agent.memory.add_message(
                    session_id=message_request.session_id,
                    role="user",
                    content=message_request.message
                )
                await agent.memory.add_message(
                    session_id=message_request.session_id,
                    role="assistant",
                    content=full_response,
                    metadata={
                        "sources": [
                            {
                                "title": result.get('source_metadata', {}).get('title', 'Unknown'),
                                "section": result.get('source_metadata', {}).get('section', 'Unknown'),
                                "page": result.get('source_metadata', {}).get('page', 'Unknown')
                            }
                            for result in retrieval_results
                        ]
                    }
                )

                # Send sources
                sources = [
                    {
                        "title": result.get('source_metadata', {}).get('title', 'Unknown'),
                        "section": result.get('source_metadata', {}).get('section', 'Unknown'),
                        "page": result.get('source_metadata', {}).get('page', 'Unknown')
                    }
                    for result in retrieval_results
                ]

                yield {
                    "event": "sources",
                    "data": json.dumps({"sources": sources})
                }

                logger.info(f"Successfully processed streaming chat request for session {message_request.session_id}")

                # Send end event
                yield {
                    "event": "end",
                    "data": json.dumps({"session_id": message_request.session_id})
                }

            except Exception as llm_error:
                logger.error(f"LLM service unavailable for session {message_request.session_id}: {str(llm_error)}", exc_info=True)

                # If the LLM service is unavailable, provide graceful degradation
                error_message = "The AI service is temporarily unavailable. Please try again later."

                yield {
                    "event": "message",
                    "data": json.dumps({"token": error_message})
                }

                yield {
                    "event": "error",
                    "data": json.dumps({
                        "error": {
                            "code": "SERVICE_UNAVAILABLE",
                            "message": str(llm_error)
                        }
                    })
                }

                # Still save user message even if LLM failed
                await agent.memory.add_message(
                    session_id=message_request.session_id,
                    role="user",
                    content=message_request.message
                )

                yield {
                    "event": "end",
                    "data": json.dumps({"session_id": message_request.session_id})
                }

        except Exception as e:
            logger.error(f"Error processing streaming chat request for session {message_request.session_id}: {str(e)}", exc_info=True)

            # Send error as Server-Sent Event
            yield {
                "event": "error",
                "data": json.dumps({
                    "error": {
                        "code": "INTERNAL_ERROR",
                        "message": f"Error processing chat request: {str(e)}"
                    }
                })
            }

    return EventSourceResponse(event_generator())