// @ts-check
import eslint from '@eslint/js'
import reactHooks from 'eslint-plugin-react-hooks'
import tseslint from 'typescript-eslint'
import { defineConfig } from 'eslint/config'


export default defineConfig(
  eslint.configs.recommended,
  tseslint.configs.recommendedTypeChecked,
  reactHooks.configs.flat['recommended-latest'],
  {
    languageOptions: {
      parserOptions: {
        projectService: true,
      },
    },
  },
  {
    files: ['src/**/*.{ts,tsx}'],
    rules: {
      "@typescript-eslint/require-await": "off",  // TODO: remove ths exemption
      'react-hooks/set-state-in-effect': 'off',  // TODO: remove ths exemption
    },
  },
  {
    ignores: ['dist/**', 'wasm-core/pkg/**', 'node_modules/**', 'vite.config.mjs'],
  },
)
