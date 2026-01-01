import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={clsx('container', styles.heroContainer)}>
        
        {/* Left Column: Text Content */}
        <div className={styles.heroContent}>
          <div className={styles.heroTagline}>Welcome to</div>
          <Heading as="h1" className={styles.heroTitle}>
            <span>Physical AI Textbook</span>
          </Heading>
          <p className={styles.heroSubtitle}>
             A complete textbook on Physical AI paired with an intelligent chatbot that helps you understand every concept clearly and quickly.
          </p>
          <div className={styles.buttons}>
            <Link
              className={styles.pillButton}
              to="/docs/intro">
              Start Reading
            </Link>
          </div>
        </div>

        {/* Removed Image Container as requested */}

      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI Textbook and Chatbot">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
