import asyncpg
import json
from typing import Optional, List, Dict, Any
from datetime import datetime
from app.core.config import settings


class NeonPostgresService:
    def __init__(self):
        self.db_url = settings.neon_db_url
        self.pool = None
    
    async def connect(self):
        """Establish connection to the Neon PostgreSQL database"""
        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.db_url,
                min_size=1,
                max_size=10,
                command_timeout=60,
                statement_cache_size=0  # Disable statement cache for serverless
            )
            # Create required tables if they don't exist
            await self._create_tables()
        except Exception as e:
            print(f"Error connecting to database: {e}")
            raise
    
    async def _create_tables(self):
        """Create tables if they don't exist"""
        async with self.pool.acquire() as conn:
            # Create sessions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS sessions (
                    id TEXT PRIMARY KEY,
                    user_id TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    last_interaction_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    active BOOLEAN DEFAULT TRUE
                )
            """)
            
            # Create messages table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS messages (
                    id SERIAL PRIMARY KEY,
                    session_id TEXT NOT NULL,
                    role TEXT NOT NULL CHECK (role IN ('user', 'assistant')),
                    content TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    metadata JSONB DEFAULT '{}',
                    FOREIGN KEY (session_id) REFERENCES sessions(id) ON DELETE CASCADE
                )
            """)
            
            # Create indexes
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_messages_session_id ON messages(session_id);
                CREATE INDEX IF NOT EXISTS idx_messages_timestamp ON messages(timestamp);
                CREATE INDEX IF NOT EXISTS idx_sessions_last_interaction ON sessions(last_interaction_at);
            """)
    
    async def create_session(self, session_id: str, user_id: Optional[str] = None):
        """Create a new session"""
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO sessions (id, user_id) 
                VALUES ($1, $2)
                ON CONFLICT (id) DO UPDATE SET
                    last_interaction_at = CURRENT_TIMESTAMP,
                    active = TRUE
                """,
                session_id, user_id
            )
    
    async def add_message(self, session_id: str, role: str, content: str, metadata: Optional[Dict[str, Any]] = None):
        """Add a message to a session"""
        if metadata is None:
            metadata = {}
        
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO messages (session_id, role, content, metadata)
                VALUES ($1, $2, $3, $4)
                """,
                session_id, role, content, json.dumps(metadata)
            )
            # Update session's last interaction time
            await conn.execute(
                """
                UPDATE sessions 
                SET last_interaction_at = CURRENT_TIMESTAMP 
                WHERE id = $1
                """,
                session_id
            )
    
    async def get_session_history(self, session_id: str) -> List[Dict[str, Any]]:
        """Get conversation history for a session"""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT role, content, timestamp, metadata 
                FROM messages 
                WHERE session_id = $1 
                ORDER BY timestamp ASC
                """,
                session_id
            )
        
        return [
            {
                "role": row["role"],
                "content": row["content"],
                "timestamp": row["timestamp"],
                "metadata": json.loads(row["metadata"]) if row["metadata"] else {}
            }
            for row in rows
        ]
    
    async def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get session details"""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT id, user_id, created_at, last_interaction_at, active FROM sessions WHERE id = $1",
                session_id
            )
        
        if row:
            return {
                "id": row["id"],
                "user_id": row["user_id"],
                "created_at": row["created_at"],
                "last_interaction_at": row["last_interaction_at"],
                "active": row["active"]
            }
        
        return None
    
    async def close(self):
        """Close the database connection pool"""
        if self.pool:
            await self.pool.close()


# Global instance
neon_service = NeonPostgresService()