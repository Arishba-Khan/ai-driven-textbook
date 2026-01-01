import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type ModuleItem = {
  step: string;
  title: string;
  description: ReactNode;
  path: string;
};

const ModuleList: ModuleItem[] = [
  {
    step: '01',
    title: 'The Robotic Nervous System (ROS 2)',
    description: (
      <>
        Understanding ROS 2 fundamentals including nodes, topics, publishers, subscribers, services, and actions for humanoid robots.
      </>
    ),
    path: '/docs/module-1/introduction-to-ros2'
  },
  {
    step: '02',
    title: 'The Digital Twin (Gazebo & Unity)',
    description: (
      <>
        Simulation environments for robotics including URDF/SDF robot descriptions, physics simulation, and sensor simulation techniques.
      </>
    ),
    path: '/docs/module-2/introduction-to-gazebo'
  },
  {
    step: '03',
    title: 'The AI-Robot Brain (NVIDIA Isaac™)',
    description: (
      <>
        AI-powered navigation and perception systems using Isaac Sim, VSLAM navigation, and sim-to-real transfer techniques.
      </>
    ),
    path: '/docs/module-3/introduction-to-nvidia-isaac-sim'
  },
  {
    step: '04',
    title: 'Vision-Language-Action (VLA) Systems',
    description: (
      <>
        Vision-Language-Action integration for cognitive robots using voice-to-action systems and LLM-based cognitive planning.
      </>
    ),
    path: '/docs/module-4/voice-to-action-openai-whisper'
  },
];

function Module({step, title, description, path}: ModuleItem) {
  return (
    <div className={clsx('col col--3', styles.moduleCard)}>
      <Link to={path} className={styles.moduleLink}>
        <div className={clsx(styles.moduleBox)}>
          <div className={styles.stepNumber}>{step}</div>
          <Heading as="h3" className={styles.moduleTitle}>{title}</Heading>
          <p className={styles.moduleDescription}>{description}</p>
        </div>
      </Link>
    </div>
  );
}

function ChatbotInfo(): ReactNode {
  return (
    <section className={styles.chatbotSection}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--12', styles.chatbotCard)}>
            <div className={styles.chatbotBox}>
              <Heading as="h2" className={styles.chatbotTitle}>AI-Powered Learning Assistant</Heading>
              <p className={styles.chatbotDescription}>
                Our RAG (Retrieval-Augmented Generation) chatbot is here to help you understand Physical AI concepts with ease.
                It has been trained specifically on this textbook content to provide accurate, contextual answers to your questions.
                Whether you need clarification on complex topics or want to dive deeper into specific subjects,
                our intelligent assistant can simplify content and guide your learning journey.
              </p>
              <div className={styles.chatbotFeatures}>
                <div className={styles.featureItem}>
                  <div className={styles.featureIcon}>🔍</div>
                  <div className={styles.featureText}>
                    <strong>Contextual Understanding</strong>
                    <p>Grasps the context of your questions based on the textbook content</p>
                  </div>
                </div>
                <div className={styles.featureItem}>
                  <div className={styles.featureIcon}>💡</div>
                  <div className={styles.featureText}>
                    <strong>Content Simplification</strong>
                    <p>Breaks down complex concepts into easy-to-understand explanations</p>
                  </div>
                </div>
                <div className={styles.featureItem}>
                  <div className={styles.featureIcon}>📚</div>
                  <div className={styles.featureText}>
                    <strong>Direct References</strong>
                    <p>Provides specific references to textbook sections for further reading</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <>
      <section className={styles.features}>
        <div className="container">
          <div className={styles.sectionHeader}>
            <Heading as="h2" className={styles.sectionTitle}>
              Let us show you how we drive your learning to new heights
            </Heading>
            <p>Master Physical AI through a structured 4-step process designed for engineers.</p>
          </div>
          <div className="row">
            {ModuleList.map((props, idx) => (
              <Module key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>
      <ChatbotInfo />
    </>
  );
}