# MapNavigation frontend

## Local development

Use Node.js 20.19+ or 22.12+, then copy `.env.example` to `.env.local` and
replace the placeholder values. Real environment files are ignored by Git.

```bash
npm ci
npm run serve
```

The Vite development server listens on <http://localhost:8080>, matching the
backend's default CORS origin. The backend API listens on
<http://localhost:8081> by default.

## Checks

```bash
npm run lint
npm run build
npm audit --omit=dev --audit-level=high
```
