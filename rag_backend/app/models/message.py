from pydantic import BaseModel
from datetime import datetime
from typing import Optional, Dict, Any


class Message(BaseModel):
    id: Optional[int] = None
    session_id: str
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime
    metadata: Optional[Dict[str, Any]] = None