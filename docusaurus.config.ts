import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the Digital Brain and the Physical Body — A Comprehensive Textbook for the Age of Embodied Intelligence',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://panaversity.github.io',
  baseUrl: '/physical-ai-textbook/',

  organizationName: 'panaversity',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو (Urdu)',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/panaversity/physical-ai-textbook/tree/main/',
          showLastUpdateTime: true,
          breadcrumbs: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/panaversity/physical-ai-textbook/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.png',
    metadata: [
      { name: 'keywords', content: 'physical AI, humanoid robotics, ROS 2, Gazebo, NVIDIA Isaac, embodied intelligence, robotics textbook' },
      { name: 'description', content: 'A comprehensive textbook for teaching Physical AI & Humanoid Robotics - covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models.' },
      { name: 'author', content: 'Muhammad Saqib' },
    ],
    colorMode: {
      defaultMode: 'dark',
      respectPrefersColorScheme: true,
    },
    announcementBar: {
      id: 'panaversity_launch',
      content: '🤖 <strong>Physical AI & Humanoid Robotics</strong> — An AI-Native Textbook by <strong>Muhammad Saqib</strong> @ <a href="https://panaversity.org" target="_blank">Panaversity</a>',
      backgroundColor: '#0a0a1a',
      textColor: '#00d4ff',
      isCloseable: true,
    },
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      style: 'dark',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: '📖 Textbook',
        },
        {
          to: '/docs/module-1/introduction-to-ros2',
          label: '🧬 Module 1: ROS 2',
          position: 'left',
        },
        {
          to: '/docs/module-2/gazebo-simulation',
          label: '🌐 Module 2: Simulation',
          position: 'left',
        },
        {
          to: '/docs/module-3/nvidia-isaac-platform',
          label: '🧠 Module 3: Isaac',
          position: 'left',
        },
        {
          to: '/docs/module-4/vision-language-action',
          label: '🗣️ Module 4: VLA',
          position: 'left',
        },
        {
          href: 'https://panaversity.org',
          label: 'Panaversity',
          position: 'right',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/panaversity/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'Module 1: ROS 2', to: '/docs/module-1/introduction-to-ros2' },
            { label: 'Module 2: Simulation', to: '/docs/module-2/gazebo-simulation' },
            { label: 'Module 3: NVIDIA Isaac', to: '/docs/module-3/nvidia-isaac-platform' },
            { label: 'Module 4: VLA', to: '/docs/module-4/vision-language-action' },
          ],
        },
        {
          title: 'Panaversity',
          items: [
            { label: 'Panaversity.org', href: 'https://panaversity.org' },
            { label: 'AI Agent Factory Book', href: 'https://ai-native.panaversity.org' },
            { label: 'PIAIC', href: 'https://piaic.org' },
            { label: 'GIAIC', href: 'https://giaic.org' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'ROS 2 Documentation', href: 'https://docs.ros.org/en/humble/' },
            { label: 'NVIDIA Isaac Sim', href: 'https://developer.nvidia.com/isaac-sim' },
            { label: 'Gazebo Sim', href: 'https://gazebosim.org/' },
            { label: 'GitHub Repository', href: 'https://github.com/panaversity/physical-ai-textbook' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Muhammad Saqib. Physical AI & Humanoid Robotics Textbook. Built with Docusaurus for Panaversity.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      // additionalLanguages: ['python', 'bash'],
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
  } satisfies Preset.ThemeConfig,

  customFields: {
    backendUrl: process.env.NEXT_PUBLIC_API_URL || 'http://localhost:3001',
    authorName: 'Muhammad Saqib',
  },
};

export default config;
