import { WebbitConfig } from '@webbitjs/webbit';
import { alertsDashboardConfig } from './alerts';
import { commandsDashboardConfig } from './commands';

export const componentElementConfigs = {
  'team1701-alerts': alertsDashboardConfig,
  'team1701-commands': commandsDashboardConfig,
} as Record<string, WebbitConfig>;
