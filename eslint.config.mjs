// @ts-check
import eslint from '@eslint/js'
import reactHooks from 'eslint-plugin-react-hooks'
import tseslint from 'typescript-eslint'

export default tseslint.config(
  eslint.configs.recommended,
  tseslint.configs.recommended,
  reactHooks.configs.flat['recommended-latest'],
  {
    files: ['src/**/*.{ts,tsx}'],
    rules: {
      // Allow unused vars that start with _ (common convention for intentional ignores)
      '@typescript-eslint/no-unused-vars': ['warn', { argsIgnorePattern: '^_', varsIgnorePattern: '^_' }],
      // Allow explicit any where needed in WASM interop and worker RPC code
      '@typescript-eslint/no-explicit-any': 'warn',
      // Allow non-null assertions (used extensively with DOM APIs and WASM)
      '@typescript-eslint/no-non-null-assertion': 'off',
      // Allow empty object types (used in type definitions)
      '@typescript-eslint/no-empty-object-type': 'off',
      // Allow namespace imports
      '@typescript-eslint/no-namespace': 'off',
      // Effects in this codebase intentionally update state in response to external events
      'react-hooks/set-state-in-effect': 'off',
    },
  },
  {
    ignores: ['dist/**', 'wasm-core/pkg/**', 'node_modules/**', 'vite.config.mjs'],
  },
)
