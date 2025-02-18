import eslint from '@eslint/js';
import importPlugin from 'eslint-plugin-import';
import prettier from 'eslint-plugin-prettier';
import tseslint from 'typescript-eslint';

export default tseslint.config(
  {
    ignores: ['**/dist', '**/bin', '**/obj', '**/cdk.out', '**/.angular', '**/TestResults'],
  },
  {
    files: ['**/*.{js,cjs,mjs,ts,cts,mts,tsx}'],
    plugins: { import: importPlugin },
    rules: {
      ...eslint.configs.recommended.rules,
      'prefer-const': ['error', { destructuring: 'all', ignoreReadBeforeAssign: false }],
      'sort-imports': ['error', { ignoreDeclarationSort: true }],
      'no-duplicate-imports': 'error',
      'import/order': ['error', { alphabetize: { order: 'asc' } }],
    },
  },
  importPlugin.flatConfigs.typescript,
  {
    plugins: { prettier: prettier },
    rules: {
      'prettier/prettier': 'error',
    },
  },
  {
    files: ['**/*.{ts,cts,mts,tsx}'],
    extends: [...tseslint.configs.recommendedTypeChecked, ...tseslint.configs.stylisticTypeChecked],
    languageOptions: {
      parserOptions: {
        project: true,
      },
    },
    rules: {
      '@typescript-eslint/consistent-indexed-object-style': 'off',
      '@typescript-eslint/no-empty-function': 'off',
      '@typescript-eslint/no-empty-interface': 'off',
      '@typescript-eslint/no-explicit-any': 'off',
      '@typescript-eslint/no-inferrable-types': ['error', { ignoreParameters: true }],
      '@typescript-eslint/no-unsafe-member-access': 'off',
      '@typescript-eslint/no-unused-vars': ['error', { args: 'none', ignoreRestSiblings: true }],
      '@typescript-eslint/no-var-requires': 'error',
      '@typescript-eslint/prefer-nullish-coalescing': 'off',
      '@typescript-eslint/unbound-method': 'off',
    },
  }
);
