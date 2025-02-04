import globals from 'globals';
import pluginJs from '@eslint/js';

/** @type {import('eslint').Linter.Config[]} */
export default [
	{
		languageOptions: {
			globals: globals.node,
		},
		rules: {
			'no-console': 'off',
			'no-unused-vars': 'warn',
			'prefer-const': 'warn',
			'no-duplicate-imports': 'error',
			'newline-before-return': 'error',
		},
	},
	{
		ignores: ['noVNC/'],
	},
	pluginJs.configs.recommended,
];
