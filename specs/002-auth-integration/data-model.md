# Data Model for All-in-One Auth Integration for Vercel

## User Entity

### Fields
- **id** (string): Unique identifier for the user (UUID)
- **email** (string): User's email address (unique, indexed)
- **password** (string): Hashed password using industry-standard hashing
- **createdAt** (datetime): Timestamp when the account was created
- **updatedAt** (datetime): Timestamp when the account was last updated
- **isEmailVerified** (boolean): Whether the email has been verified (default: false)

### Extended Profile Fields
- **experienceLevel** (string): User's experience level (e.g., "beginner", "intermediate", "advanced")
- **programmingLanguages** (string): Comma-separated list of programming languages the user knows
- **roboticsExperience** (string): User's experience with robotics
- **gpuAvailable** (boolean): Whether the user has access to a GPU
- **hardwareAccess** (string): Description of hardware available to the user

## Session Entity

### Fields
- **id** (string): Unique identifier for the session (UUID)
- **userId** (string): Reference to the user who owns this session
- **token** (string): Session token (securely generated)
- **expiresAt** (datetime): Expiration timestamp (30 days from creation)
- **createdAt** (datetime): Timestamp when the session was created
- **lastAccessedAt** (datetime): Timestamp when the session was last used

## Relationships
- **User** 1 ←→ * **Session**: One user can have multiple active sessions

## Validation Rules
- **Email**: Must be a valid email format and unique across all users
- **Password**: Must be at least 8 characters with mixed case, number, and special character
- **experienceLevel**: Must be one of "beginner", "intermediate", "advanced", or null
- **programmingLanguages**: Can be a comma-separated list of programming languages
- **gpuAvailable**: Boolean value only
- **createdAt/updatedAt**: Automatically managed by the database

## State Transitions
- **User Registration**: User entity is created with `isEmailVerified` set to false
- **Email Verification**: User's `isEmailVerified` is updated to true
- **Session Creation**: New session is created when user successfully authenticates
- **Session Expiration**: Session is removed when `expiresAt` is reached or user logs out
- **Profile Update**: User can update their profile information (including extended fields) after authentication