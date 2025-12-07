import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

function ChapterCard({ title, description, to }) {
  return (
    <div className="col col--4 margin-bottom--lg">
      <div className="card">
        <div className="card__body">
          <h3>{title}</h3>
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--primary" to={to}>
            Read More
          </Link>
        </div>
      </div>
    </div>
  );
}

function ChapterSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2>Chapters</h2>
          </div>
        </div>
        <div className="row">
          <ChapterCard
            title="Module 1: The Robotic Nervous System (ROS 2)"
            description="Learn about ROS 2, the middleware that enables communication between different robotic components and systems."
            to="/docs/module1/"
          />
          <ChapterCard
            title="Module 2: The Digital Twin (Gazebo & Unity)"
            description="Explore simulation environments that create digital replicas of physical robots for testing and development."
            to="/docs/module2/"
          />
          <ChapterCard
            title="Module 3: The AI-Robot Brain (NVIDIA Isaac)"
            description="Discover how AI frameworks power robot decision-making and autonomous behavior."
            to="/docs/module3/"
          />
        </div>
        <div className="row">
          <ChapterCard
            title="Module 4: Vision-Language-Action (VLA)"
            description="Understand how robots integrate vision, language, and action for complex tasks."
            to="/docs/capstone/"
          />
          <ChapterCard
            title="Quick Start"
            description="Get started quickly with practical examples and hands-on exercises."
            to="/docs/quickstart"
          />
          <ChapterCard
            title="Conclusion"
            description="Wrap up your learning journey with insights and next steps."
            to="/docs/conclusion"
          />
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive guide to Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <ChapterSection />
      </main>
    </Layout>
  );
}