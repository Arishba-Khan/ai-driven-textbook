import React from 'react';
import { AuthProvider } from '../components/auth/AuthProvider';

import ChatWidget from '../components/ChatWidget';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
