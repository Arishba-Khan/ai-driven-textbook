from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from app.api.chat import router as chat_router
from app.core.config import settings
from app.core.memory import NeonPostgresMemory
from app.models.error import ErrorResponse, ErrorDetails
import asyncio
import logging
from datetime import datetime


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def create_app() -> FastAPI:
    app = FastAPI(
        title="RAG Chatbot Backend Service",
        description="RAG Chatbot Backend Service for Physical AI & Humanoid Robotics Textbook",
        version="1.0.0"
    )

    # Add CORS middleware for Docusaurus frontend
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, replace with specific domain
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routers
    app.include_router(chat_router, prefix="/chat", tags=["chat"])

    @app.get("/health")
    async def health_check():
        return {"status": "healthy", "timestamp": "2025-12-17T10:30:00Z"}

    # Error handling middleware
    @app.exception_handler(Exception)
    async def global_exception_handler(request: Request, exc: Exception):
        logger.error(f"Unhandled exception: {exc}", exc_info=True)

        error_response = ErrorResponse(
            error=ErrorDetails(
                code="INTERNAL_ERROR",
                message="An internal server error occurred",
                details=str(exc) if settings.debug_mode else None
            )
        )

        return JSONResponse(
            status_code=500,
            content=error_response.model_dump()
        )

    # Background task to cleanup expired sessions
    @app.on_event("startup")
    async def start_cleanup_task():
        async def cleanup_loop():
            from app.core.neon_service import neon_service
            memory = NeonPostgresMemory()
            while True:
                try:
                    await memory.cleanup_expired_sessions()
                    # Run cleanup every hour
                    await asyncio.sleep(3600)
                except Exception as e:
                    logger.error(f"Error during session cleanup: {e}", exc_info=True)

        # Create the background task
        asyncio.create_task(cleanup_loop())

    return app


# Create the main FastAPI app instance
app = create_app()