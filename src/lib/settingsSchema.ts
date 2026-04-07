import { CONTROLLER_SETTINGS_SECTIONS } from './settingsControllerSections';
import { RUNTIME_AND_UI_SETTINGS_SECTIONS } from './settingsRuntimeSections';

export type { SettingsFieldDefinition, SettingsSectionDefinition } from './settingsSchemaBase';

export const SETTINGS_SECTIONS = [...CONTROLLER_SETTINGS_SECTIONS, ...RUNTIME_AND_UI_SETTINGS_SECTIONS];
