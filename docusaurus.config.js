// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

const config = {
  title: 'Physical AI & Humanoid Robotics — Essentials',
  tagline: 'Learn Physical AI and Humanoid Robotics with AI-powered assistance',
  favicon: 'img/favicon.ico',
  url: 'https://hooriaarshad.github.io',
  baseUrl: '/',
  organizationName: 'HooriaArshad',
  projectName: 'docasuros-book',
  onBrokenLinks: 'throw',
  markdown: {
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
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],
  themeConfig: ({
    // Move sidebar to left side
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
    navbar: {
      title: '🤖 Physical AI & Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjZmZmZmZmIiBzdHJva2Utd2lkdGg9IjIiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgc3Ryb2tlLWxpbmVqb2luPSJyb3VuZCI+PHBhdGggZD0iTTEyIDIwYTggOCAwIDEgMCAwLTE2IDggOCAwIDAgMCAwIDE2eiIvPjxwYXRoIGQ9Ik05IDEyaDZhMiAyIDAgMCAxIDIgMmgyLjVhMiAyIDAgMCAxIDIgMnYyaC0xMHYtMmgtMiIvPjxwYXRoIGQ9Ik03IDEwdjZoMnYtaC0yeiIvPjxwYXRoIGQ9Ik0xNyAxMHY2aC0ydi02aDJ6Ii8+PHBhdGggZD0iTTkgNnYySDZ2LTJoM3ptMTAgMGgyVjRoM3YyeiIvPjxwYXRoIGQ9Ik05IDJ2MmgtMnYtMmgyem0xMCAwaDJ2MmgtMnYtMmgyeiIvPjwvc3ZnPg==',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: '📚 Textbook',
        },
        {
          href: 'https://www.instagram.com/call_me_hoora_/',
          label: '📷 Instagram',
          position: 'left',
        },
        {
          href: 'https://github.com/HooriaArshad/docasuros-book',
          label: '⭐ GitHub',
          position: 'right',
        },
        {
          href: 'https://www.linkedin.com/in/hooria-arshad-hoor-ba32b5301/',
          label: '💼 LinkedIn',
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
            {
              label: '📖 Preface',
              to: '/preface',
            },
            {
              label: 'Chapter 1: Physical AI',
              to: '/chapters/physical-ai',
            },
            {
              label: 'Chapter 2: Humanoid Robotics',
              to: '/chapters/humanoid-robotics',
            },
            {
              label: 'Chapter 6: Capstone',
              to: '/chapters/capstone',
            },
          ],
        },
        {
          title: 'Author',
          items: [
            {
              label: 'Created by: Hooria Arshad',
              to: '/',
            },
            {
              label: '📷 Instagram: @call_me_hoora_',
              href: 'https://www.instagram.com/call_me_hoora_/',
            },
            {
              label: '💼 LinkedIn: Hooria Arshad',
              href: 'https://www.linkedin.com/in/hooria-arshad-hoor-ba32b5301/',
            },
            {
              label: '⭐ GitHub: @HooriaArshad',
              href: 'https://github.com/HooriaArshad',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'Gazebo Simulator',
              href: 'https://gazebosim.org/',
            },
            {
              label: 'NVIDIA Isaac Sim',
              href: 'https://developer.nvidia.com/isaac-sim',
            },
          ],
        },
      ],
      copyright: 'Copyright 2025 Physical AI Textbook by Hooria Arshad. Built with Docusaurus.',
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  }),
};

export default config;
