import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/auth/SigninForm';

const SigninPage: React.FC = () => {
  return (
    <Layout title="Sign In to Physical AI Textbook" description="Sign in to your account for the Physical AI & Humanoid Robotics textbook">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Sign In to Physical AI Textbook</h1>
            <p>
              Access the Physical AI & Humanoid Robotics textbook and personalized features.
            </p>
            <SigninForm />
            <div className="margin-top--lg">
              <p>Don't have an account? <a href="/signup">Sign up here</a>.</p>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SigninPage;