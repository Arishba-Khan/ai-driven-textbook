import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/auth/SigninForm';

const SigninPage: React.FC = () => {
  return (
    <Layout title="Sign In to Physical AI Textbook" description="Sign in to your account for the Physical AI & Humanoid Robotics textbook">
      <div className="container margin-vert--lg">
        <SigninForm />
      </div>
    </Layout>
  );
};

export default SigninPage;