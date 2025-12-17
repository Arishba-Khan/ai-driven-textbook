import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

// Define the User interface
export interface User {
  id: string;
  email: string;
  name?: string;
  experienceLevel?: string;
  hardwareAccess?: string;
  programmingLanguages?: string;
  roboticsExperience?: string;
  gpuAvailable?: boolean;
  createdAt: string;
  updatedAt: string;
}

// Define the AuthContext type
interface AuthContextType {
  user: User | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (userData: {
    email: string;
    password: string;
    name?: string;
    experienceLevel?: string;
    hardwareAccess?: string;
    programmingLanguages?: string;
    roboticsExperience?: string;
    gpuAvailable?: boolean;
  }) => Promise<void>;
  signOut: () => Promise<void>;
}

// Create the AuthContext
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Define props for AuthProvider
interface AuthProviderProps {
  children: ReactNode;
}

// Define API base URL - use localhost:3001 for development if not proxied
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost' 
  ? 'http://localhost:3001/api/auth' 
  : '/api/auth';

// AuthProvider component
export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Check for existing session on component mount
  useEffect(() => {
    const checkSession = async () => {
      try {
        const response = await fetch(`${API_BASE_URL}/get-session`, {
          credentials: 'include',
          headers: {
            'Content-Type': 'application/json',
          },
        });

        if (response.ok) {
          const data = await response.json();
          // Handle both null responses and object responses
          if (data && data.session && data.user) {
            setUser(data.user);
          } else {
            setUser(null);
          }
        } else {
          setUser(null);
        }
      } catch (error) {
        console.error('Error checking session:', error);
        setUser(null);
      } finally {
        setLoading(false);
      }
    };

    // Only check session if we're in a browser environment
    if (typeof window !== 'undefined') {
      checkSession();
    }
  }, []);

  // Sign in function
  const signIn = async (email: string, password: string) => {
    try {
      const response = await fetch(`${API_BASE_URL}/sign-in/email`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        let errorMessage = 'Login failed';
        try {
          const error = await response.json();
          errorMessage = error.message || errorMessage;
        } catch (e) {
          // Response is not JSON, use default message
        }
        throw new Error(errorMessage);
      }

      const data = await response.json();
      if (data.user) {
        setUser(data.user);
      }
      return data;
    } catch (error) {
      console.error('Sign in error:', error);
      throw error;
    }
  };

  // Sign up function
  const signUp = async (userData: {
    email: string;
    password: string;
    name?: string;
    experienceLevel?: string;
    hardwareAccess?: string;
    programmingLanguages?: string;
    roboticsExperience?: string;
    gpuAvailable?: boolean;
  }) => {
    try {
      const response = await fetch(`${API_BASE_URL}/sign-up/email`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData),
      });

      if (!response.ok) {
        let errorMessage = 'Registration failed';
        try {
          const error = await response.json();
          errorMessage = error.message || errorMessage;
        } catch (e) {
          // Response is not JSON, use default message
        }
        throw new Error(errorMessage);
      }

      const data = await response.json();
      if (data.user) {
        setUser(data.user);
      }
      return data;
    } catch (error) {
      console.error('Sign up error:', error);
      throw error;
    }
  };

  // Sign out function
  const signOut = async () => {
    try {
      await fetch(`${API_BASE_URL}/sign-out`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      setUser(null);
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  const value = {
    user,
    loading,
    signIn,
    signUp,
    signOut,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use the AuthContext
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};