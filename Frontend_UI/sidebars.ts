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
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-robot-control',
        'module-1/chapter-3-robot-body-representation',
        'module-1/validation-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation (Gazebo & Unity)',
      items: [
        'module-2/chapter-1-gazebo-physics',
        'module-2/chapter-2-unity-interaction',
        'module-2/chapter-3-sensor-simulation',
        'module-2/validation-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/chapter-1-isaac-sim',
        'module-3/chapter-2-isaac-ros',
        'module-3/chapter-3-nav2-humanoids',
        'module-3/validation-summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/chapter-1-voice-to-action',
        'module-4/chapter-2-cognitive-planning',
        'module-4/chapter-3-capstone-humanoid',
        'module-4/validation-summary',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
