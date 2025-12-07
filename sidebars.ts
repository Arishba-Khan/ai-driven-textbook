import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'intro/why-physical-ai',
        'intro/quarter-overview',
        'intro/learning-outcomes',
        'intro/weekly-breakdown'
      ],
      link: {
        type: 'doc',
        id: 'intro/index'
      }
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/introduction-to-ros2',
        'module-1/ros2-nodes-deep-dive',
        'module-1/topics-and-publishers',
        'module-1/subscribers-and-callbacks',
        'module-1/services-request-response',
        'module-1/actions-long-running-tasks',
        'module-1/urdf-for-humanoid-robots'
      ],
      link: {
        type: 'generated-index',
        title: 'Module 1: The Robotic Nervous System (ROS 2)',
        description: 'Understanding ROS 2 fundamentals',
        slug: '/module-1'
      }
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/introduction-to-gazebo',
        'module-2/urdf-sdf-robot-description',
        'module-2/physics-simulation-gravity-collisions',
        'module-2/sensor-simulation-lidar-depth-imu',
        'module-2/unity-high-fidelity-rendering'
      ],
      link: {
        type: 'generated-index',
        title: 'Module 2: The Digital Twin (Gazebo & Unity)',
        description: 'Simulation environments for robotics',
        slug: '/module-2'
      }
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/introduction-to-nvidia-isaac-sim',
        'module-3/isaac-ros-vslam-navigation',
        'module-3/nav2-path-planning-bipedal',
        'module-3/sim-to-real-transfer-techniques'
      ],
      link: {
        type: 'generated-index',
        title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
        description: 'AI-powered navigation and perception systems',
        slug: '/module-3'
      }
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/voice-to-action-openai-whisper',
        'module-4/llm-cognitive-planning-natural-language',
        'module-4/capstone-autonomous-humanoid'
      ],
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action (VLA)',
        description: 'Vision-Language-Action integration for cognitive robots',
        slug: '/module-4'
      }
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware/index'
      ],
      link: {
        type: 'generated-index',
        title: 'Hardware Requirements',
        description: 'Complete hardware specifications and options',
        slug: '/hardware'
      }
    }
  ],
};

export default sidebars;
