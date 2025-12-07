# Code Example Format Skill

This skill defines the exact structure and quality standards for all code examples included in the "Physical AI & Humanoid Robotics" textbook.

## Standard Template (Copy-Paste Ready)

Every code example MUST follow this template:

````markdown
```python
# [Brief description of what the code does]
# Code goes here
```

**To Run This Code:**

1.  [Step 1: e.g., Save as `example.py`]
2.  [Step 2: e.g., Run `python example.py` in your terminal]

**Expected Output:**

```
[Exact output shown here]
```

**Explanation:**

[2-4 sentences explaining *why* the code works the way it does, not just *what* it does.]

**Key Takeaway:**

[1 sentence summarizing the main lesson from this example.]
````

## Code Requirements

*   **Completeness:** All code examples must be complete, self-contained, and runnable.
*   **Readability:** Code must be clean, well-formatted, and easy to understand.
*   **Comments:** Include inline comments every 3-5 lines to explain non-obvious logic.
*   **Language Standards:** Adhere to language-specific style guides (e.g., PEP 8 for Python, ESLint for JavaScript/TypeScript).
*   **Error Handling:** Basic error handling should be present where appropriate, but complex error handling can be simplified for clarity.
*   **Dependencies:** Clearly state any external dependencies and how to install them.

## Output Requirements

*   **Accuracy:** The "Expected Output" section MUST show EXACTLY what the user will see in their terminal when running the code.
*   **Formatting:** Use code blocks for output to preserve formatting.

## Explanation Requirements

*   **Focus on "Why":** The explanation should focus on the underlying principles, concepts, and *why* specific choices were made, rather than just reiterating *what* the code does.
*   **Conciseness:** Keep explanations brief and to the point (2-4 sentences).

## Key Takeaway Requirements

*   **Single Sentence:** A single, impactful sentence that summarizes the main learning point or conclusion from the example.

## Final Validation Checklist (for Agents)

When creating or reviewing a code example, ensure it meets the following:

*   [ ] Adheres to the "Standard Template"
*   [ ] Code is complete and runnable
*   [ ] Code is well-commented (every 3-5 lines)
*   [ ] Follows language-specific style guides
*   [ ] Includes basic error handling where appropriate
*   [ ] States external dependencies clearly
*   [ ] "Expected Output" is exact and correctly formatted
*   [ ] "Explanation" focuses on "why" and is concise
*   [ ] "Key Takeaway" is a single, summary sentence
*   [ ] Can I copy-paste and run this immediately without modification?
*   [ ] Is the code production-ready (within the context of an example)?
