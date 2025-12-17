import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { AuthProvider } from '@site/src/components/auth/AuthProvider';

type LayoutProps = {
  children: React.ReactNode;
};

const Layout: React.FC<LayoutProps> = ({ children }) => {
  return (
    <AuthProvider>
      <OriginalLayout>{children}</OriginalLayout>
    </AuthProvider>
  );
};

export default Layout;