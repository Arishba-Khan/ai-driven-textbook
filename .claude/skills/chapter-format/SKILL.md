# Chapter Format Skill

This skill defines the exact structure every chapter in the "Physical AI & Humanoid Robotics" textbook must follow to ensure consistency in structure, formatting, tone, and quality across all content.

## Structure and Naming Conventions

1.  **File Naming:** Chapters must be named `##-topic-name.md` (e.g., `01-introduction-to-robotics.md`).
2.  **YAML Frontmatter:** Every chapter file must begin with YAML frontmatter, including at least `title` and `description` fields.

    ```yaml
    ---
    title: "Chapter Title"
    description: "A brief description of the chapter."
    ---
    ```

## Required Sections (in order)

Each chapter MUST contain the following sections, using the specified Markdown headings:

### 1. Title (H1)

*   The main title of the chapter.

### 2. Learning Objectives (H2)

*   A bulleted list of 3-5 clear, measurable learning objectives.
    *   Example: "Explain the fundamental concepts of…"

### 3. Introduction (H2)

*   (200-300 words) Introduce the chapter's topic, its relevance, and what the reader will learn.

### 4. Core Concepts (H2)

*   (800-1200 words) Detailed explanation of the main concepts.
*   Use H3 subheadings for individual concepts.
*   Include illustrative examples and analogies.
*   Mermaid diagrams should be used to visualize abstract concepts, workflows, or architectures. The agent should generate the Mermaid code directly within the Markdown file.

### 5. Practical Examples (H2)

*   (400-600 words) Provide 1-2 practical code examples that demonstrate the core concepts.
*   Each example should be self-contained and runnable.
*   Include clear explanations for each part of the code.

### 6. Hands-on Exercise (H2)

*   (150-250 words) A small, actionable exercise for the reader to apply what they've learned.
*   Include clear instructions and expected outcomes.
*   Do NOT include solutions directly in the chapter; solutions will be in a separate section of the textbook.

### 7. Common Pitfalls (H2)

*   (100-200 words) Discuss common mistakes or misunderstandings related to the chapter's topic.

### 8. Summary (H2)

*   (100-150 words) Recap the main points and learning objectives.

### 9. Further Reading (H2)

*   A bulleted list of 2-4 external resources (books, articles, documentation) for deeper dives.

## Content Guidelines

*   **Word Count:** Each chapter should be approximately 1500-2500 words (excluding code blocks).
*   **Tone & Style:** Conversational yet professional, engaging, and easy to understand. Use the second person ("you") to address the reader directly.
*   **Technical Accuracy:** All information must be accurate and up-to-date.
*   **Clarity:** Avoid jargon where possible; explain technical terms clearly.

## Quality Checklist for Agents

When creating or reviewing a chapter, ensure it meets the following:

*   [ ] Adheres to file naming convention (##-topic-name.md)
*   [ ] Includes correct YAML frontmatter
*   [ ] All 9 required sections are present and in order
*   [ ] Learning objectives are clear and measurable
*   [ ] Introduction is engaging and sets context
*   [ ] Core Concepts are well-explained with H3 subheadings and Mermaid diagrams where appropriate
*   [ ] Practical Examples are clear, runnable, and well-explained
*   [ ] Hands-on Exercise is actionable and clear (no solutions provided)
*   [ ] Common Pitfalls are relevant and helpful
*   [ ] Summary effectively recaps the chapter
*   [ ] Further Reading links are relevant and high-quality
*   [ ] Chapter meets word count guidelines
*   [ ] Tone and style are consistent
*   [ ] Technical information is accurate
*   [ ] Content is clear and easy to understand

## Usage Instructions for Agents

When asked to create a new chapter or review an existing one, an agent should:

1.  **Plan:** First, use the `Plan` tool to outline the chapter content based on this skill's guidelines.
2.  **Draft:** Use the `Write` tool to draft the chapter content section by section.
3.  **Review:** Use the `Task` tool with an appropriate subagent (e.g., `code-reviewer` for code sections, `general-purpose` for content review) to ensure all guidelines and the quality checklist are met.
4.  **Iterate:** Refine the chapter based on review feedback until it fully complies with this skill.
