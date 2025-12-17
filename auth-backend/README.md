# Auth Backend for Physical AI & Humanoid Robotics Textbook

This backend service provides authentication functionality for the Physical AI & Humanoid Robotics textbook, using Better Auth with custom user fields for background information.

## Features

- User registration with background information (experience level, hardware access, programming languages, etc.)
- User login and session management
- Health check endpoint
- Integration with Neon PostgreSQL database
- Vercel serverless deployment ready

## Custom User Fields

The authentication system captures these additional fields during registration:
- `experienceLevel`: User's experience level with robotics
- `programmingLanguages`: Comma-separated list of programming languages the user knows
- `roboticsExperience`: User's experience with robotics
- `gpuAvailable`: Whether the user has access to a GPU
- `hardwareAccess`: Description of hardware available to the user

## Environment Variables

Create a `.env` file with the following variables:

```env
DATABASE_URL="your_neon_database_url"
AUTH_SECRET="your_auth_secret_key"
FRONTEND_URL="https://your-frontend-url.vercel.app"
```

For local development:
```env
FRONTEND_URL="http://localhost:3000"
```

## Scripts

- `npm run dev`: Start development server with nodemon
- `npm run build`: Compile TypeScript to JavaScript
- `npm start`: Start the production server

## API Endpoints

- `POST /api/auth/register`: Register a new user
- `POST /api/auth/login`: Login a user
- `POST /api/auth/logout`: Logout a user
- `GET /api/auth/me`: Get current user info
- `PATCH /api/auth/profile`: Update user profile
- `GET /api/health`: Health check endpoint

## Deployment

This backend is designed to run as a Vercel serverless function and is configured in the root `vercel.json` file. When deployed with the frontend, both run in the same domain to eliminate CORS issues.