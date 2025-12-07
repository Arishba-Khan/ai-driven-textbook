# Research: Physical AI & Humanoid Robotics Textbook

## Decision: Docusaurus Version
**Rationale**: Docusaurus v3 is the latest stable version with modern React features, TypeScript support, and excellent documentation capabilities. It's specifically designed for documentation sites and textbooks like this project.
**Alternatives considered**: 
- Docusaurus v2 (stable but older)
- GitBook (less customization options)
- Hugo (would require learning Go templating)

## Decision: Content Structure
**Rationale**: Organizing content in the `docs/` directory with module-specific subdirectories follows Docusaurus conventions and matches the syllabus structure perfectly.
**Alternatives considered**:
- Flattened structure (would be harder to navigate)
- Different directory names (would be inconsistent with syllabus)

## Decision: Code Example Format
**Rationale**: Using Python code examples targeting ROS 2 Humble on Ubuntu 22.04 matches the syllabus requirements and is appropriate for students with Python knowledge.
**Alternatives considered**:
- C++ examples (would be too difficult for target audience)
- Different ROS versions (Humble is the current LTS version)

## Decision: Content Chapter Structure
**Rationale**: Following the required structure (Overview → Core Concepts → Hands-on Examples → Key Takeaways → Quick Exercises) ensures comprehensive learning.
**Alternatives considered**:
- Different structures (would not meet syllabus requirements)

## Decision: Hardware Table Format
**Rationale**: Using clean Markdown tables exactly as provided in the syllabus meets the verbatim requirement.
**Alternatives considered**:
- Different table formats (would not match syllabus exactly)

## Decision: Visual Elements
**Rationale**: Using Mermaid diagrams for technical concepts and appropriate admonitions (note, tip, warning, info) for emphasis aligns with the technical standards.
**Alternatives considered**:
- Different diagramming tools (Mermaid is built into Docusaurus)
- Different emphasis mechanisms (admonitions are standard Docusaurus feature)

## Decision: Code Tabs
**Rationale**: Using Docusaurus code tabs for multi-language examples ensures clean presentation where applicable.
**Alternatives considered**:
- Separate code blocks (less organized)
- Other tab solutions (Docusaurus native solution is preferred)

## Decision: Build and Deployment
**Rationale**: Deploying to GitHub Pages or Vercel is specified in the requirements and provides appropriate hosting for a static documentation site.
**Alternatives considered**:
- Self-hosting (unnecessary complexity)
- Other platforms (GitHub Pages/Vercel are standard options)