from typing import Protocol, List, Dict, Any, Optional
from app.core.neon_service import neon_service
from app.models.message import Message
from datetime import datetime, timedelta
import asyncio


class SessionProtocol(Protocol):
    async def create_session(self, session_id: str, user_id: Optional[str] = None):
        ...

    async def add_message(self, session_id: str, role: str, content: str, metadata: Optional[Dict[str, Any]] = None):
        ...

    async def get_session_history(self, session_id: str) -> List[Dict[str, Any]]:
        ...

    async def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        ...


class NeonPostgresMemory:
    def __init__(self):
        self.neon_service = neon_service
        # Session timeout in hours (default 24 hours of inactivity)
        self.session_timeout_hours = 24

    async def create_session(self, session_id: str, user_id: Optional[str] = None):
        """
        Create a new session in the database.

        Args:
            session_id: Unique identifier for the session
            user_id: Optional user identifier
        """
        await self.neon_service.create_session(session_id, user_id)

    async def add_message(self, session_id: str, role: str, content: str, metadata: Optional[Dict[str, Any]] = None):
        """
        Add a message to the session history.

        Args:
            session_id: Session identifier
            role: 'user' or 'assistant'
            content: Message content
            metadata: Optional metadata dictionary
        """
        await self.neon_service.add_message(session_id, role, content, metadata)

    async def get_session_history(self, session_id: str) -> List[Message]:
        """
        Retrieve the conversation history for a session.

        Args:
            session_id: Session identifier

        Returns:
            List of Message objects in chronological order
        """
        history = await self.neon_service.get_session_history(session_id)
        messages = []
        for item in history:
            message = Message(
                session_id=session_id,
                role=item['role'],
                content=item['content'],
                timestamp=item['timestamp'],
                metadata=item['metadata']
            )
            messages.append(message)
        return messages

    async def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """
        Get session details.

        Args:
            session_id: Session identifier

        Returns:
            Session details dictionary or None if not found
        """
        session_data = await self.neon_service.get_session(session_id)

        # Check if session has expired
        if session_data and not self.is_session_active(session_data):
            # Mark session as inactive
            await self.mark_session_inactive(session_id)
            session_data['active'] = False

        return session_data

    def is_session_active(self, session_data: Dict[str, Any]) -> bool:
        """
        Check if a session is still active based on last interaction time.

        Args:
            session_data: Session data dictionary containing last_interaction_at

        Returns:
            True if session is active, False otherwise
        """
        if not session_data.get('active', False):
            return False

        last_interaction = session_data['last_interaction_at']
        timeout_duration = timedelta(hours=self.session_timeout_hours)
        current_time = datetime.now(last_interaction.tzinfo) if last_interaction.tzinfo else datetime.now()

        return (current_time - last_interaction) < timeout_duration

    async def mark_session_inactive(self, session_id: str):
        """
        Mark a session as inactive in the database.

        Args:
            session_id: Session identifier
        """
        async with self.neon_service.pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE sessions
                SET active = FALSE
                WHERE id = $1
                """,
                session_id
            )

    async def cleanup_expired_sessions(self):
        """
        Mark expired sessions as inactive. This should be run periodically.
        """
        async with self.neon_service.pool.acquire() as conn:
            timeout_duration = timedelta(hours=self.session_timeout_hours)
            expired_cutoff = datetime.now() - timeout_duration

            await conn.execute(
                """
                UPDATE sessions
                SET active = FALSE
                WHERE active = TRUE AND last_interaction_at < $1
                """,
                expired_cutoff
            )