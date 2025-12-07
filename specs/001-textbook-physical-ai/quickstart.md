# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Getting Started

This guide will help you get started with the Physical AI & Humanoid Robotics textbook project. This is a comprehensive Docusaurus v3-based documentation site designed for students with Python and basic AI knowledge but no robotics experience.

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git version control system
- A modern web browser

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-textbook
   ```

2. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

3. Start the development server:
   ```bash
   npm start
   # or
   yarn start
   ```

4. Open your browser to [http://localhost:3000](http://localhost:3000) to view the textbook.

## Project Structure

The textbook is organized into the following modules:

- **Intro**: Introduction to Physical AI, quarter overview, learning outcomes, and weekly breakdown
- **Module 1**: The Robotic Nervous System (ROS 2) - 7 chapters covering fundamentals
- **Module 2**: The Digital Twin (Gazebo & Unity) - 5 chapters on simulation
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac™) - 4 chapters on navigation and AI
- **Module 4**: Vision-Language-Action (VLA) - 3 chapters including the capstone project
- **Hardware**: Complete hardware requirements with all 6 tables from the syllabus

## Creating New Content

### Chapter Structure

Each chapter must follow this exact structure:

```markdown
---
title: "Chapter Title"
description: "Brief description for SEO"
---

## Learning Objectives

- Understand the purpose of [topic]
- Implement [specific skill]
- Analyze [specific concept]

## Introduction

Provide a 200-300 word introduction that hooks the student and explains why this topic matters in the context of Physical AI and humanoid robotics.

## Core Concepts

Cover the main concepts in 500-800 words with H3 subheadings for organization.

### Subheading Example

Detailed explanation of the concept with clear examples.

## Hands-on Examples

Include 2-3 complete, runnable Python code examples with:

- Clear comments every 3-5 lines
- Expected output shown
- Explanation of what the code demonstrates

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="python" label="Python" default>

```python
# Complete, executable Python code example
print("Hello, ROS 2!")
```

</TabItem>
</Tabs>

Expected Output:
```
Hello, ROS 2!
```

## Exercises

Include 1-2 hands-on exercises with success criteria:

- [ ] Criterion 1
- [ ] Criterion 2
- [ ] Criterion 3

## Common Pitfalls and Solutions

- **Pitfall 1**: Explanation of common mistake and how to avoid it
- **Pitfall 2**: Another common issue and solution

## Summary

- Key takeaway 1
- Key takeaway 2
- Key takeaway 3

## Further Reading

- [Resource 1](link)
- [Resource 2](link)
```

### Adding a New Chapter

1. Create a new Markdown file in the appropriate module directory
2. Follow the naming convention: `XX-chapter-title.md` (e.g., `01-introduction-to-ros2.md`)
3. Use the structure outlined above
4. Update the `sidebars.ts` file to include your new chapter in the navigation

## Hardware Tables

For the hardware chapter, reproduce all 6 tables exactly as provided in the syllabus:

1. Digital Twin Workstation Table
2. Physical AI Edge Kit Table
3. Robot Lab Options A/B/C Table
4. Summary of Architecture Table
5. Cloud-Native "Ether" Lab Table
6. Economy Jetson Student Kit Table

## Building for Production

To build the textbook for deployment:

```bash
npm run build
# or
yarn build
```

The static files will be generated in the `build/` directory and can be deployed to any static hosting service.

## Key Principles to Follow

1. **Clarity First**: Ensure all explanations are accessible to students with Python and basic AI knowledge but no robotics experience
2. **Encouraging Tone**: Maintain a warm, mentor-like voice throughout
3. **Hands-on Mandate**: Include complete, copy-paste-ready code examples with expected output
4. **Zero Fluff**: Every word must teach - no filler content
5. **Verbatim Requirements**: Hardware tables must match syllabus exactly
6. **Complete Coverage**: Ensure 100% syllabus content is represented

## Testing Your Changes

1. Verify the development server runs without errors
2. Test all code examples in your chapters
3. Check mobile responsiveness
4. Ensure all links work correctly
5. Validate that the site builds successfully with `npm run build`

## Deployment

The textbook is designed to deploy to GitHub Pages or Vercel. For GitHub Pages, run:

```bash
GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
```

For Vercel, follow their standard deployment process.

## Need Help?

- Check the existing chapters for examples of proper formatting
- Refer to the Docusaurus documentation for component options
- Review the constitution document for project principles and standards