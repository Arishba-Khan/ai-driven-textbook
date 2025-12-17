# Quickstart Guide: All-in-One Auth Integration for Vercel

## Prerequisites

- Node.js 18+ installed
- Git installed
- A Vercel account
- A Neon PostgreSQL account

## Setup Instructions

### 1. Clone and Navigate to the Repository
```bash
git clone <your-repo-url>
cd <repo-name>
```

### 2. Install Dependencies
```bash
# Install root dependencies
npm install

# Install auth-backend dependencies
cd auth-backend
npm install
cd ..
```

### 3. Set Up Environment Variables
Create a `.env` file in the root directory with the following:
```env
DATABASE_URL="your_neon_database_url"
AUTH_SECRET="your_auth_secret_key"
FRONTEND_URL="http://localhost:3000"  # For local development
```

### 4. Run the Development Server
```bash
# Run both frontend and backend simultaneously
npm run dev:all
```

The Docusaurus frontend will be available at `http://localhost:3000` and the auth backend will be available at `http://localhost:3001`.

### 5. Running Tests
```bash
# Run backend tests
cd auth-backend && npm test

# Run frontend tests
npm test
```

## Key Features

### User Registration
- Visit `/signup` to register with your email and background information
- The form collects all required background information fields
- All data is validated before submission

### User Login
- Visit `/signin` to log in with your credentials
- Session management handles authentication state

### Protected Content
- The translation button is only visible to authenticated users
- Additional personalized content based on user's background information

### API Endpoints
- `/api/auth/register` - User registration
- `/api/auth/login` - User login
- `/api/auth/logout` - User logout
- `/api/auth/me` - Get current user info
- `/api/auth/profile` - Update user profile
- `/api/health` - Health check endpoint

## Deployment to Vercel

### 1. Set Environment Variables in Vercel Dashboard
Add these environment variables in your Vercel project settings:
- `DATABASE_URL`
- `AUTH_SECRET`
- `FRONTEND_URL` (set to your Vercel deployment URL)

### 2. Deploy
Simply push your changes to the connected GitHub repository:
```bash
git add .
git commit -m "Deploy auth integration"
git push origin main
```

Vercel will automatically deploy both the frontend and backend according to the configuration in `vercel.json`.

## Troubleshooting

### Common Issues
- If the auth endpoints return CORS errors during development, ensure your frontend and backend are running on the correct ports
- If database connection fails, verify your `DATABASE_URL` is correct
- If the auth provider isn't working, verify that the `AuthProvider` is wrapping the entire app in `src/theme/Layout/index.tsx`

### Development Tips
- Keep session timeouts reasonable during development (30 days in production, but can be shorter in development)
- Use the health check endpoint (`/api/health`) to verify backend status during development
- Test both authenticated and unauthenticated flows when developing new features