import type { SettingsValueFormat } from './settingsUnits';

export type SettingsFieldDefinition = {
  path: readonly string[];
  label: string;
  description: string;
  step: number;
  unit: string;
  toDisplay: (value: number) => number;
  toStored: (value: number) => number;
  min?: number;
};

export type SettingsSectionDefinition = {
  title: string;
  description: string;
  fields: SettingsFieldDefinition[];
};

export function field(
  path: readonly string[],
  label: string,
  description: string,
  step: number,
  valueFormat: SettingsValueFormat,
  min?: number,
) {
  return { path, label, description, step, min, ...valueFormat } satisfies SettingsFieldDefinition;
}
