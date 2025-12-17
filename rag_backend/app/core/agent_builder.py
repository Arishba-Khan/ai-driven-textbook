import groq
from typing import Dict, Any, List
import time
import logging
from app.core.config import settings
from app.core.retriever import QdrantRetriever
from app.core.memory import NeonPostgresMemory


# Set up logging
logger = logging.getLogger(__name__)


class Agent:
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.groq_client = groq.Groq(api_key=settings.groq_api_key)
        self.retriever = QdrantRetriever()
        self.memory = NeonPostgresMemory()
        self.max_retries = 3
        self.retry_delay = 1  # seconds

    def _make_groq_request_with_retry(self, messages, model="llama-3.1-8b-instant", temperature=0.7, max_tokens=1024):
        """
        Make a request to Groq API with retry logic.
        """
        for attempt in range(self.max_retries):
            try:
                response = self.groq_client.chat.completions.create(
                    messages=messages,
                    model=model,
                    temperature=temperature,
                    max_tokens=max_tokens
                )
                return response
            except Exception as e:
                logger.warning(f"Groq API call failed (attempt {attempt + 1}): {e}")
                if attempt == self.max_retries - 1:  # Last attempt
                    raise e
                time.sleep(self.retry_delay * (2 ** attempt))  # Exponential backoff

    def _make_qdrant_request_with_retry(self, query, limit=5):
        """
        Make a request to Qdrant with retry logic.
        """
        for attempt in range(self.max_retries):
            try:
                return self.retriever.retrieve(query, limit)
            except Exception as e:
                logger.warning(f"Qdrant API call failed (attempt {attempt + 1}): {e}")
                if attempt == self.max_retries - 1:  # Last attempt
                    raise e
                time.sleep(self.retry_delay * (2 ** attempt))  # Exponential backoff

    async def invoke(self, message: str, context_text: str = None) -> Dict[str, Any]:
        """
        Process a user message and return a response from the LLM,
        using RAG to ground the response in textbook content.
        """
        try:
            # Retrieve relevant context from the textbook with retry logic
            retrieval_results = self._make_qdrant_request_with_retry(message)

            # Get conversation history
            history = await self.memory.get_session_history(self.session_id)

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
                "content": message
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

            # Call the LLM with retry logic
            response = self._make_groq_request_with_retry(
                messages=chat_history,
                model="llama-3.1-8b-instant",  # Using Groq's llama3.1 model
                temperature=0.7,
                max_tokens=1024
            )

            # Extract the response
            assistant_response = response.choices[0].message.content

            # Save the interaction to memory
            await self.memory.add_message(
                session_id=self.session_id,
                role="user",
                content=message
            )
            await self.memory.add_message(
                session_id=self.session_id,
                role="assistant",
                content=assistant_response,
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

            # Extract sources for response
            sources = [
                {
                    "title": result.get('source_metadata', {}).get('title', 'Unknown'),
                    "section": result.get('source_metadata', {}).get('section', 'Unknown'),
                    "page": result.get('source_metadata', {}).get('page', 'Unknown')
                }
                for result in retrieval_results
            ]

            return {
                "response": assistant_response,
                "session_id": self.session_id,
                "sources": sources
            }

        except Exception as e:
            logger.error(f"Error in agent invocation: {e}", exc_info=True)
            return {
                "response": "Sorry, I encountered an error processing your request. Please try again later.",
                "session_id": self.session_id,
                "sources": []
            }


def create_agent(session_id: str) -> Agent:
    """
    Factory function to create an agent instance for a specific session.
    
    Args:
        session_id: The session identifier
        
    Returns:
        Agent instance configured for the session
    """
    return Agent(session_id)