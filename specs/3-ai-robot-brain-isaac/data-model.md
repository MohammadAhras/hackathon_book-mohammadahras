# Data Model: The AI-Robot Brain (NVIDIA Isaac)

## Documentation Entities

### Chapter Entity
- **Name**: Isaac Sim and Synthetic Worlds
- **Fields**:
  - title: string
  - description: string
  - learning_objectives: array of strings
  - content_sections: array of section references
  - prerequisites: array of prerequisite references
- **Relationships**: belongs to Module 3, contains multiple articles

### Chapter Entity
- **Name**: Isaac ROS and Visual SLAM
- **Fields**:
  - title: string
  - description: string
  - learning_objectives: array of strings
  - content_sections: array of section references
  - prerequisites: array of prerequisite references
- **Relationships**: belongs to Module 3, contains multiple articles

### Chapter Entity
- **Name**: Nav2 Navigation Stack
- **Fields**:
  - title: string
  - description: string
  - learning_objectives: array of strings
  - content_sections: array of section references
  - prerequisites: array of prerequisite references
- **Relationships**: belongs to Module 3, contains multiple articles

### Article Entity
- **Fields**:
  - title: string
  - slug: string
  - content: string (Markdown)
  - frontmatter: object (YAML metadata)
  - learning_objectives: array of strings
  - code_examples: array of code blocks
- **Relationships**: belongs to a Chapter

### Learning Objective Entity
- **Fields**:
  - description: string
  - level: enum (beginner, intermediate, advanced)
  - verification_method: string
- **Relationships**: belongs to Chapter/Article

### Code Example Entity
- **Fields**:
  - language: string
  - code: string
  - purpose: string
  - usage_context: string
- **Relationships**: belongs to Article

## Validation Rules
- Each Chapter must have at least 3 learning objectives
- Each Article must have proper Docusaurus frontmatter
- All content must follow Flesch-Kincaid grade 10-12 readability standards
- All code examples must be syntactically correct
- All articles must have proper navigation integration

## State Transitions
- Draft → Review (when content is written)
- Review → Approved (when reviewed and validated)
- Approved → Published (when integrated into documentation site)