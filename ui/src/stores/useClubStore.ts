import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface CustomClub {
  id: string;
  name: string;
  base_type: string;
  loft_deg: number;
  enabled: boolean;
}

export interface ClubsSnapshot {
  clubs: CustomClub[];
  lofts: Record<string, number>;
}

interface ClubState extends ClubsSnapshot {
  loaded: boolean;
  useMyClubs: boolean;
  setUseMyClubs: (value: boolean) => void;
  applySnapshot: (snapshot: ClubsSnapshot) => void;
}

export const useClubStore = create<ClubState>()(
  persist(
    (set) => ({
      clubs: [],
      lofts: {},
      loaded: false,
      useMyClubs: false,
      setUseMyClubs: (useMyClubs) => set({ useMyClubs }),
      applySnapshot: (snapshot) => set({ ...snapshot, loaded: true }),
    }),
    { name: 'openflight-club-picker', partialize: (state) => ({ useMyClubs: state.useMyClubs }) }
  )
);
