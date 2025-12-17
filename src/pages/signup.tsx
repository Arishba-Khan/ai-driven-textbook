import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/auth/SignupForm';

const SignupPage: React.FC = () => {
  return (
    <Layout title="Sign Up for Physical AI Textbook" description="Create an account for the Physical AI & Humanoid Robotics textbook">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Sign Up for Physical AI Textbook</h1>
            <p>
              Create an account to access the Physical AI & Humanoid Robotics textbook and personalized features.
              Please provide your background information to help us tailor the content to your needs.
            </p>
            <SignupForm />
            <div className="margin-top--lg">
              <p>Already have an account? <a href="/signin">Sign in here</a>.</p>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SignupPage;