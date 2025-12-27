// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar:  [
    {
      type: 'doc',
      id: 'intro',
    
    },
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'ros2-fundamentals/intro',
        'ros2-fundamentals/nodes-topics-services-actions',
        'ros2-fundamentals/communication-model',
        'ros2-fundamentals/role-in-physical-ai',
      ],

    },
    {
      type: 'category',
      label: 'Humanoid Modeling with URDF',
      collapsed: false,
      items: [
        'humanoid-modeling/intro',
        'humanoid-modeling/links-joints-sensors',
        'humanoid-modeling/representing-anatomy',
        'humanoid-modeling/urdf-simulation-control',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation',
      items: [
        'digital-twin-simulation/intro',
        {
          type: 'category',
          label: 'Physics Simulation with Gazebo',
          items: [
            'digital-twin-simulation/physics-simulation-gazebo/intro',
            'digital-twin-simulation/physics-simulation-gazebo/gravity-collisions-dynamics',
            'digital-twin-simulation/physics-simulation-gazebo/world-robot-simulation',
            'digital-twin-simulation/physics-simulation-gazebo/role-gazebo-robotics-testing',
          ],
        },
        {
          type: 'category',
          label: 'High-Fidelity Environments with Unity',
          items: [
            'digital-twin-simulation/high-fidelity-unity/intro',
            'digital-twin-simulation/high-fidelity-unity/visual-realism-interaction',
            'digital-twin-simulation/high-fidelity-unity/human-robot-interaction-scenarios',
            'digital-twin-simulation/high-fidelity-unity/unity-gazebo-integration',
          ],
        },
        {
          type: 'category',
          label: 'Sensor Simulation',
          items: [
            'digital-twin-simulation/sensor-simulation/index',  // This one has index.md
            'digital-twin-simulation/sensor-simulation/lidar-depth-cameras-imus',
            'digital-twin-simulation/sensor-simulation/sensor-noise-realism',
            'digital-twin-simulation/sensor-simulation/simulation-reality-considerations',
          ],
        },
      ],

    },
    {
      type: 'category',
      label: 'The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'ai-robot-brain-isaac/index',
        {
          type: 'category',
          label: 'Isaac Sim and Synthetic Worlds',
          items: [
            'ai-robot-brain-isaac/isaac-sim-synthetic-worlds/index',
            'ai-robot-brain-isaac/isaac-sim-synthetic-worlds/isaac-sim-overview',
            'ai-robot-brain-isaac/isaac-sim-synthetic-worlds/synthetic-environments',
            'ai-robot-brain-isaac/isaac-sim-synthetic-worlds/sensor-simulation',
            'ai-robot-brain-isaac/isaac-sim-synthetic-worlds/domain-randomization',
          ],
        },
        {
          type: 'category',
          label: 'Isaac ROS and Visual SLAM',
          items: [
            'ai-robot-brain-isaac/isaac-ros-visual-slam/index',
            'ai-robot-brain-isaac/isaac-ros-visual-slam/isaac-ros-packages',
            'ai-robot-brain-isaac/isaac-ros-visual-slam/visual-slam-pipelines',
            'ai-robot-brain-isaac/isaac-ros-visual-slam/camera-calibration',
            'ai-robot-brain-isaac/isaac-ros-visual-slam/3d-reconstruction',
          ],
        },
        {
          type: 'category',
          label: 'Nav2 Navigation Stack',
          items: [
            'ai-robot-brain-isaac/nav2-navigation-stack/index',
            'ai-robot-brain-isaac/nav2-navigation-stack/nav2-architecture',
            'ai-robot-brain-isaac/nav2-navigation-stack/path-planning-humanoids',
            'ai-robot-brain-isaac/nav2-navigation-stack/costmap-configuration',
            'ai-robot-brain-isaac/nav2-navigation-stack/behavior-trees-navigation',
          ],
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      items: [
        'vla-module/index',
        {
          type: 'category',
          label: 'Voice-to-Action with Speech Models',
          items: [
            'vla-module/voice-to-action-with-speech-models/index',
          ],
        },
        {
          type: 'category',
          label: 'Language-Driven Cognitive Planning',
          items: [
            'vla-module/language-driven-cognitive-planning/index',
          ],
        },
        {
          type: 'category',
          label: 'Capstone: The Autonomous Humanoid',
          items: [
            'vla-module/autonomous-humanoid-capstone/index',
          ],
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Docusaurus UI Upgrade',
      items: [
        'docusaurus-ui/index',
        'docusaurus-ui/summary-next-steps',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;