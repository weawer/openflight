import { test } from '@playwright/test';
import { expect, gotoApp, KIOSK_VIEWPORTS, simulateShot, waitForEvent, withControlSocket } from './helpers';

for (const viewport of KIOSK_VIEWPORTS) {
  test.describe(`custom clubs ${viewport.width}x${viewport.height}`, () => {
    test.use({ viewport, hasTouch: true });

    test.beforeEach(async () => {
      await withControlSocket(async (socket) => {
        const snapshot = waitForEvent<{ clubs: { id: string }[] }>(socket, 'clubs');
        socket.emit('get_clubs');
        for (const club of (await snapshot).clubs) {
          await socket.emitWithAck('remove_custom_club', { id: club.id });
        }
      });
    });

    test('create, select, reload, edit and delete custom clubs', async ({ page }) => {
      await gotoApp(page);
      await page.getByRole('checkbox', { name: 'Use my clubs' }).check();
      await expect(page.getByText('No custom clubs yet.')).toBeVisible();
      await page.getByRole('button', { name: 'Add club', exact: true }).tap();
      await page.getByRole('button', { name: 'Name', exact: true }).tap();
      await page.getByRole('textbox', { name: 'Club name' }).fill('My iron');
      await page.getByRole('button', { name: 'Club name', exact: true }).tap();
      await page.getByRole('combobox', { name: 'Type', exact: true }).selectOption('7-iron');
      await page.getByRole('spinbutton', { name: 'Loft' }).fill('30');
      await expect(page.getByRole('button', { name: 'Save', exact: true })).toBeInViewport();
      await page.getByRole('button', { name: 'Save', exact: true }).tap();
      await page.getByRole('button', { name: 'My iron 7 Iron' }).tap();
      await expect(page.getByRole('dialog', { name: 'Select club' })).toHaveCount(0);
      const result = (await withControlSocket((socket) => simulateShot(socket))) as {
        shot: { club: string; custom_club_name: string; custom_club_id: string };
      };
      expect(result.shot.club).toBe('7-iron');
      expect(result.shot.custom_club_name).toBe('My iron');
      expect(result.shot.custom_club_id).toBeTruthy();
      await page.reload();
      await expect(page.getByRole('checkbox', { name: 'Use my clubs' })).toBeChecked();
      await expect(page.getByRole('button', { name: 'My iron 7 Iron' })).toHaveAttribute('aria-pressed', 'true');
      await page.getByRole('button', { name: 'Edit My iron' }).tap();
      await page.getByRole('button', { name: 'Name', exact: true }).tap();
      await page.getByRole('textbox', { name: 'Club name' }).fill('Renamed iron');
      await page.getByRole('button', { name: 'Club name', exact: true }).tap();
      await page.getByRole('button', { name: 'Save', exact: true }).tap();
      await page.getByRole('button', { name: 'Edit Renamed iron' }).tap();
      await page.getByRole('button', { name: 'Delete club' }).tap();
      await expect(page.getByText('No custom clubs yet.')).toBeVisible();
      await page.getByRole('checkbox', { name: 'Use my clubs' }).uncheck();
      await expect(page.getByRole('button', { name: '7i', exact: true })).toHaveAttribute('aria-pressed', 'true');
    });

    test('drag scrolls without selecting and touch tap selects one club', async ({ page }) => {
      await withControlSocket(async (socket) => {
        for (let i = 0; i < 20; i++) {
          await socket.emitWithAck('save_custom_club', {
            name: `Iron ${String(i).padStart(2, '0')}`,
            base_type: '7-iron',
            loft_deg: 30,
          });
        }
      });
      await gotoApp(page);
      await page.getByRole('checkbox', { name: 'Use my clubs' }).check();
      const list = page.locator('.custom-clubs__list');
      await expect(list.locator('.custom-clubs__select')).toHaveCount(20);
      const box = (await list.boundingBox())!;
      await page.mouse.move(box.x + 80, box.y + box.height - 25);
      await page.mouse.down();
      await page.mouse.move(box.x + 80, box.y + 20, { steps: 12 });
      await page.mouse.up();
      await expect.poll(() => list.evaluate((el) => el.scrollTop)).toBeGreaterThan(30);
      await expect(page.getByRole('dialog', { name: 'Select club' })).toBeVisible();
      await list.evaluate((el) => {
        el.scrollTop = 0;
      });
      await page.getByRole('button', { name: 'Iron 00 7 Iron' }).tap();
      await expect(page.getByRole('dialog', { name: 'Select club' })).toHaveCount(0);
      await page.reload();
      await expect(page.getByRole('button', { name: 'Iron 00 7 Iron' })).toHaveAttribute('aria-pressed', 'true');
    });
  });
}
