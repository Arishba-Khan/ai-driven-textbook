# Data Model: Physical AI & Humanoid Robotics Textbook

## Content Entities

### Chapter
- **name**: String - The chapter's unique identifier (e.g. "01-introduction-to-ros2")
- **title**: String - The human-readable title
- **module**: String - Which module the chapter belongs to (intro, module-1, module-2, module-3, module-4, hardware)
- **position**: Number - Order within the module
- **learningObjectives**: Array[String] - 3-5 action-verb based learning objectives
- **introduction**: String - 200-300 words with hook and why it matters
- **coreConcepts**: String - 500-800 words with H3 subheadings
- **codeExamples**: Array[CodeExample] - 2-3 runnable Python code examples
- **exercises**: Array[Exercise] - 1-2 hands-on exercises with success criteria
- **diagrams**: Array[MermaidDiagram] - At least 1 Mermaid diagram
- **pitfalls**: Array[String] - Common pitfalls and solutions (2-3)
- **summary**: Array[String] - Key takeaways (3-5 bullet points)
- **furtherReading**: Array[String] - 3-5 resources for deeper learning
- **wordCount**: Number - Total word count (800-1500 excluding code)

### CodeExample
- **title**: String - Brief description of the example
- **language**: String - Programming language (always "python" for this project)
- **code**: String - Complete, runnable code with comments every 3-5 lines
- **expectedOutput**: String - What output students should see when running the code
- **purpose**: String - Explanation of what the example demonstrates

### Exercise
- **title**: String - Brief description of the exercise
- **description**: String - Detailed instructions for the student
- **successCriteria**: Array[String] - 3-5 checkboxes of what constitutes success
- **difficulty**: String - "beginner", "intermediate", or "advanced"

### MermaidDiagram
- **title**: String - Brief description of the diagram
- **type**: String - "flowchart", "sequence", "class", etc.
- **code**: String - Mermaid syntax code for the diagram
- **caption**: String - Explanation of what the diagram illustrates

### HardwareTable
- **title**: String - Table title
- **columns**: Array[String] - Column headers
- **rows**: Array[Array[String]] - Table data
- **source**: String - Where this table came from in the syllabus

## Navigation Entities

### Module
- **id**: String - Unique identifier (module-1, module-2, etc.)
- **title**: String - Display title
- **description**: String - Brief overview of the module
- **chapters**: Array[Chapter] - Ordered list of chapters in this module

### Sidebar
- **items**: Array[MenuItem] - Navigation items in order
- **labels**: Object - Human-readable labels for navigation

### MenuItem
- **type**: String - "doc" or "category"
- **id**: String - Identifier for the documentation page or category
- **label**: String - Display name for the menu item
- **items**: Array[MenuItem] - Child items if this is a category

## Validation Rules

### Chapter Validation
- Length: Must be between 800-1500 words (excluding code)
- Structure: Must follow exact chapter structure (Overview → Core Concepts → Hands-on Examples → Key Takeaways → Quick Exercises)
- Content: All terminology must be bolded and explained immediately on first use
- Code: Must include 2-3 runnable Python examples with expected output
- Exercises: Must include 1-2 hands-on exercises with success criteria
- Diagrams: Must include at least 1 Mermaid diagram
- Tables: Hardware tables must reproduce syllabus content verbatim

### Module Validation
- Complete Coverage: Each module must cover all topics specified in syllabus
- Difficulty Progression: Must progress from simple to complex concepts
- ROS 2 Prerequisites: Module 1 must provide sufficient foundation for later modules

### Hardware Table Validation
- Verbatim: Must reproduce exactly as provided in syllabus
- Format: Must be in clean Markdown format
- Completeness: All 6 required tables must be included