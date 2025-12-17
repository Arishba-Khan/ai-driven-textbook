from pydantic import BaseModel
from datetime import datetime
from typing import Optional


class Session(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime
    last_interaction_at: datetime
    active: bool = True