// @ts-check
import eslint from '@eslint/js'
import tseslint from 'typescript-eslint'

export default tseslint.config(
  eslint.configs.recommended,
  tseslint.configs.recommended,
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
    },
  },
  {
    ignores: ['dist/**', 'wasm-core/pkg/**', 'node_modules/**', 'vite.config.mjs'],
  },
)
