# Data Model: ROS 2 Book Module 1

## Content Entities

### Chapter Document
- **name**: String (required) - The chapter identifier (e.g., "chapter-1-ros2-fundamentals")
- **title**: String (required) - The display title of the chapter
- **content**: String (required) - The markdown content of the chapter
- **authors**: Array[String] - Authors of the chapter content
- **reviewers**: Array[String] - Reviewers who validated the content
- **citations**: Array[Citation] - References used in the chapter
- **diagrams**: Array[Diagram] - Conceptual diagrams included in the chapter
- **metadata**: Object - Additional metadata like creation date, last updated, etc.

### Citation
- **id**: String (required) - Unique identifier for the citation
- **type**: String (required) - Type of source (e.g., "journal", "book", "website", "paper")
- **title**: String (required) - Title of the source
- **authors**: Array[String] - Authors of the source
- **year**: Number (required) - Publication year
- **url**: String (optional) - URL if available
- **doi**: String (optional) - DOI for academic papers
- **apa_format**: String (required) - Full APA format citation string

### Diagram
- **id**: String (required) - Unique identifier for the diagram
- **title**: String (required) - Title/description of the diagram
- **filename**: String (required) - Name of the file in static assets
- **alt_text**: String (required) - Alternative text for accessibility
- **description**: String (required) - Detailed description of the diagram content
- **category**: String (required) - Category (e.g., "architecture", "workflow", "structure")

### Module
- **id**: String (required) - Module identifier (e.g., "module-1")
- **title**: String (required) - Display title of the module
- **chapters**: Array[Chapter Document] - Ordered list of chapters in the module
- **target_audience**: Array[String] - Target audience for the module (e.g., "computer science students", "AI engineers")
- **learning_objectives**: Array[String] - Learning objectives for the module
- **prerequisites**: Array[String] - Prerequisites for understanding the module

## Validation Rules from Requirements

### From FR-001: Docusaurus-based book module
- Each module must have at least one chapter document
- Module title must be non-empty
- Chapters must be ordered in the module

### From FR-002: Conceptual diagrams
- Each chapter document may have zero or more diagrams
- Diagrams must have alt_text for accessibility
- Diagram filenames must exist in static assets

### From FR-007: Suitable for target audience
- Module must specify target audience
- Content must be appropriate for computer science students and AI engineers

### From FR-008: Markdown format
- Content must be in valid markdown format
- Content must be compatible with Docusaurus markdown processing

## Relationships

- Module 1:N Chapter Document (one module contains multiple chapters)
- Chapter Document 1:N Citation (one chapter can have multiple citations)
- Chapter Document 1:N Diagram (one chapter can have multiple diagrams)

## State Transitions (if applicable)

For content review workflow:
- DRAFT → UNDER_REVIEW → APPROVED → PUBLISHED
- APPROVED → NEEDS_UPDATE → DRAFT (if changes required)