# Quickstart: ROS 2 Book Module 1

## Setup Instructions

### Prerequisites
- Node.js 18.x or higher
- npm or yarn package manager
- Git for version control

### 1. Initialize Docusaurus Project
```bash
# Navigate to your project directory
cd /path/to/your/project

# Install Docusaurus
npm init docusaurus@latest my-website classic

# Or using yarn
yarn create docusaurus my-website classic
```

### 2. Create Module Directory and Chapter Files
```bash
# Create the module directory
mkdir -p docs/module-1

# Create the three chapter files
touch docs/module-1/chapter-1-ros2-fundamentals.md
touch docs/module-1/chapter-2-robot-control.md
touch docs/module-1/chapter-3-robot-body-representation.md
```

### 3. Add Content to Chapter Files

#### Chapter 1: ROS 2 Fundamentals
Add content to `docs/module-1/chapter-1-ros2-fundamentals.md`:
```markdown
---
sidebar_position: 1
title: Chapter 1 - ROS 2 Fundamentals
---

# Chapter 1: ROS 2 Fundamentals

[Content will be added based on specification requirements]
```

#### Chapter 2: Robot Control
Add content to `docs/module-1/chapter-2-robot-control.md`:
```markdown
---
sidebar_position: 2
title: Chapter 2 - Controlling Robots with ROS 2
---

# Chapter 2: Controlling Robots with ROS 2

[Content will be added based on specification requirements]
```

#### Chapter 3: Robot Body Representation
Add content to `docs/module-1/chapter-3-robot-body-representation.md`:
```markdown
---
sidebar_position: 3
title: Chapter 3 - Robot Body Representation
---

# Chapter 3: Robot Body Representation

[Content will be added based on specification requirements]
```

### 4. Configure Sidebar Navigation
Update `sidebars.js` to include the new module:
```javascript
// sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-robot-control',
        'module-1/chapter-3-robot-body-representation',
      ],
    },
  ],
};
```

### 5. Add Static Assets (Diagrams)
```bash
# Create directory for images
mkdir -p static/img

# Add your diagrams to the static/img directory
# Examples:
# static/img/ros2-architecture.svg
# static/img/nodes-topics-services-actions.svg
# static/img/urdf-structure.png
```

### 6. Install Additional Dependencies (if needed)
```bash
# Install any additional Docusaurus plugins
npm install @docusaurus/plugin-content-docs
```

### 7. Run the Development Server
```bash
# Start the development server
npm run start

# Or with yarn
yarn start
```

### 8. Build for Production
```bash
# Build the static site
npm run build

# Or with yarn
yarn build
```

## Academic Compliance Checklist

### Before Publishing Content
- [ ] Content meets Flesch-Kincaid grade 10-12 readability
- [ ] At least 50% of sources are peer-reviewed or official
- [ ] All claims have APA-style citations
- [ ] Content passes plagiarism check
- [ ] Diagrams have proper alt text for accessibility
- [ ] All functional requirements from spec are satisfied

## Deployment to GitHub Pages

### 1. Configure Deployment Settings
Update `docusaurus.config.js` with GitHub Pages settings:
```javascript
module.exports = {
  // ... other config
  organizationName: 'your-username', // Usually your GitHub org/user name
  projectName: 'your-project-name', // Usually your repo name
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
};
```

### 2. Deploy
```bash
# Deploy to GitHub Pages
npm run deploy

# Or with yarn
yarn deploy
```

## Troubleshooting

### Common Issues
- **Build fails**: Check for markdown syntax errors
- **Images not showing**: Verify files are in `static/` directory
- **Sidebar not updating**: Restart development server after sidebar changes
- **Citations not rendering**: Ensure proper markdown formatting for citations