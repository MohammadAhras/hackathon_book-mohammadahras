// @ts-nocheck
const { themes } = require('prism-react-renderer');

const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn ROS 2 fundamentals, Python agents with rclpy, and humanoid modeling with URDF',
  favicon: 'img/favicon.ico',

  // ✅ Vercel deployment configuration
  url: 'http://localhost:3000', // Your Vercel deployment URL
  baseUrl: '/', // Serve from root - correct for Vercel
  // trailingSlash: false, // Recommended for Vercel

  organizationName: 'MohammadAhras',
  projectName: 'hackathon_book-mohammadahras',

  onBrokenLinks: 'warn', // Changed from 'throw' to avoid build failures during development
  markdown: {
    format: 'detect',
    mermaid: true,
    // Move the deprecated option to the new location
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // For Vercel deployment, keep docs at /docs route
          routeBasePath: 'docs', // This serves docs at /docs/
        },
        blog: false, // Disable blog to avoid route conflicts
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'ROS 2 Robotics Module',
      logo: {
        alt: 'ROS 2 Robotics Module Logo',
        src: 'img/logo.svg',
        href: '/', // Link to homepage
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Docs',
        },
        {
          href: 'https://github.com/MohammadAhras',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'ROS 2 Fundamentals', to: '/docs/ros2-fundamentals/intro' },
            { label: 'Humanoid Modeling', to: '/docs/humanoid-modeling/intro' },
            { label: 'Digital Twin Simulation', to: '/docs/digital-twin-simulation/intro' },
            { label: 'AI-Robot Brain (Isaac)', to: '/docs/ai-robot-brain-isaac' },
            { label: 'Vision-Language-Action', to: '/docs/vla-module' },
            { label: 'Docusaurus UI Upgrade', to: '/docs/docusaurus-ui' },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/MohammadAhras/hackathon_book-mohammadahras',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Robotics Module. Built with Docusaurus.`,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  },
};

module.exports = config;



// // @ts-nocheck

// const { themes } = require('prism-react-renderer');

// const lightCodeTheme = themes.github;
// const darkCodeTheme = themes.dracula;

// /** @type {import('@docusaurus/types').Config} */
// const config = {
//   title: 'Physical AI & Humanoid Robotics',
//   tagline: 'Learn ROS 2 fundamentals, Python agents with rclpy, and humanoid modeling with URDF',
//   favicon: 'img/favicon.ico',

//   url: 'https://MohammadAhras.github.io',
//   baseUrl: '/Hackathon_book/',

//   organizationName: 'qmobx',
//   projectName: 'Hackathon_book',

//   onBrokenLinks: 'ignore',
//   markdown: {
//     format: 'detect',
//     hooks: {
//       onBrokenMarkdownLinks: 'warn',
//     },
//   },

//   i18n: {
//    defaultLocale: 'en',
//     locales: ['en'],
//   },

//   presets: [
//     [
//       'classic',
//       {
//        docs: {
//         routeBasePath: 'docs',
//         sidebarPath: require.resolve('./sidebars.js'),
//           },
//         blog: false,  // Disable blog to avoid route conflicts, or set path: '/blog'
//         theme: {
//           customCss: require.resolve('./src/css/custom.css'),
//         },
//       },
//     ],
//   ],

//   themeConfig: {
//   image: 'img/docusaurus-social-card.jpg',

//   navbar: {
//     title: 'ROS 2 Robotics Module',
//     logo: {
//       alt: 'ROS 2 Robotics Module Logo',
//       src: 'img/logo.svg',
//       href: '/',
//     },
//     items: [
//       {
//         type: 'docSidebar',
//         sidebarId: 'tutorialSidebar',
//         position: 'left',
//         label: 'Docs',
//       },
//       {
//         href: 'https://github.com/MohammadAhras',
//         label: 'GitHub',
//         position: 'right',
//       },
//     ],
//   },

//   footer: {
//     style: 'dark',
//     links: [
//       {
//         title: 'Docs',
//         items: [
//           { label: 'Introduction', to: '/docs/intro' },
//           { label: 'ROS 2 Fundamentals', to: '/docs/ros2-fundamentals/intro' },
//           { label: 'Humanoid Modeling', to: '/docs/humanoid-modeling/intro' },
//           { label: 'Digital Twin Simulation', to: '/docs/digital-twin-simulation/intro' },
//           { label: 'AI-Robot Brain (Isaac)', to: '/docs/ai-robot-brain-isaac/index' },
//           { label: 'Vision-Language-Action', to: '/docs/vla-module/index' },
//           { label: 'Docusaurus UI Upgrade', to: '/docs/docusaurus-ui/index' },
//         ],
//       },
//       {
//         title: 'Community',
//         items: [
//           {
//             label: 'Stack Overflow',
//             href: 'https://stackoverflow.com/questions/tagged/docusaurus',
//           },
//           {
//             label: 'Discord',
//             href: 'https://discordapp.com/invite/docusaurus',
//           },
//         ],
//       },
//       {
//         title: 'More',
//         items: [
//           {
//             label: 'GitHub',
//             href: 'https://github.com/MohammadAhras/hackathon_book-mohammadahras',
//           },
//         ],
//       },
//     ],
//     copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Robotics Module. Built with Docusaurus.`,
//   },

//   prism: {
//     theme: require('prism-react-renderer').themes.github,
//     darkTheme: require('prism-react-renderer').themes.dracula,
//   },
// },


// }

// module.exports = config