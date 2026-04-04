import { useState, type MouseEvent } from 'react';

import {
  createDefaultAppConfig,
  getNumericAppConfigValue,
  updateNumericAppConfigValue,
  type AppConfig,
} from '../lib/appConfig';
import { SETTINGS_SECTIONS } from '../lib/settingsSchema';

type SettingsPanelProps = {
  isOpen: boolean;
  config: AppConfig;
  hasChanges: boolean;
  onConfigChange: (config: AppConfig) => void;
  onClose: () => void;
  onReset: () => void;
};

function stopPanelClick(event: MouseEvent<HTMLElement>) {
  event.stopPropagation();
}

export function SettingsPanel({ isOpen, config, hasChanges, onConfigChange, onClose, onReset }: SettingsPanelProps) {
  const [activeSectionIndex, setActiveSectionIndex] = useState(0);

  if (!isOpen) {
    return null;
  }

  const defaultConfig = createDefaultAppConfig();
  const sectionEntries = [
    ...SETTINGS_SECTIONS.map((section) => ({
      title: section.title,
      description: section.description,
      render: () => (
        <div className="settings-grid">
          {section.fields.map((field) => {
            const value = getNumericAppConfigValue(config, field.path);
            const defaultValue = getNumericAppConfigValue(defaultConfig, field.path);
            const fieldId = field.path.join('-');
            const hintId = `${fieldId}-hint`;
            return (
              <label key={field.path.join('.')} className="settings-field">
                <span className="settings-field__label">{field.label}</span>
                <input
                  id={fieldId}
                  type="number"
                  value={String(value)}
                  min={field.min}
                  step={field.step}
                  aria-describedby={hintId}
                  onChange={(event) => {
                    if (event.target.value.trim() === '') {
                      return;
                    }
                    const nextValue = Number(event.target.value);
                    if (!Number.isFinite(nextValue)) {
                      return;
                    }
                    onConfigChange(updateNumericAppConfigValue(config, field.path, nextValue));
                  }}
                />
                <span id={hintId} className="settings-field__hint">
                  {field.description} Default: {defaultValue}
                </span>
              </label>
            );
          })}
        </div>
      ),
    })),
    {
      title: 'About',
      description: 'Project links and repository information.',
      render: () => (
        <a
          className="settings-link"
          href="https://github.com/FredBill1/AutonomousDrivingDemoWASM"
          target="_blank"
          rel="noreferrer"
        >
          github.com/FredBill1/AutonomousDrivingDemoWASM
        </a>
      ),
    },
  ];
  const activeSection = sectionEntries[activeSectionIndex] ?? sectionEntries[0];

  return (
    <div className="settings-overlay" onClick={onClose}>
      <aside className="settings-modal" onClick={stopPanelClick}>
        <div className="settings-panel__header">
          <div>
            <h2>Settings</h2>
            <p>Changes are saved in local storage and applied when this panel closes.</p>
          </div>
          <button className="ghost-button" onClick={onClose}>
            {hasChanges ? 'Apply & Close' : 'Close'}
          </button>
        </div>

        <div className="settings-panel__content">
          <nav className="settings-nav" aria-label="Settings sections">
            {sectionEntries.map((section, index) => (
              <button
                key={section.title}
                className={`settings-nav__button ${index === activeSectionIndex ? 'active' : ''}`}
                onClick={() => setActiveSectionIndex(index)}
              >
                {section.title}
              </button>
            ))}
          </nav>

          <section className="settings-section settings-section--active">
            <div className="settings-section__heading">
              <h3>{activeSection.title}</h3>
              <p>{activeSection.description}</p>
            </div>
            {activeSection.render()}
          </section>
        </div>

        <div className="settings-panel__footer">
          <button className="ghost-button" onClick={onReset}>
            Reset to defaults
          </button>
          <button className="accent-button" onClick={onClose}>
            {hasChanges ? 'Apply settings' : 'Done'}
          </button>
        </div>
      </aside>
    </div>
  );
}
