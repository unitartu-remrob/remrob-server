import globals from 'globals';
import pluginJs from '@eslint/js';

/** @type {import('eslint').Linter.Config[]} */
export default [
	{
		languageOptions: {
			globals: globals.node,
		},
		rules: {
			'no-unused-vars': 'warn',
			'no-console': 'off',
			'prefer-const': 'warn',
			'no-unused-imports': 'error',
			'no-duplicate-imports': 'error',
			'newline-before-return': 'error',
		},
	},
	{
		ignores: ['noVNC/'],
	},
	pluginJs.configs.recommended,
];
