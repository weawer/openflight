import { useRef, useState } from 'react';
import { ALL_CLUBS, getClubName } from '../../data/clubs';
import { useDragScroll } from '../../hooks/useDragScroll';
import { useI18n } from '../../i18n/useI18n';
import { socketService } from '../../services/socketService';
import { useClubStore, type CustomClub } from '../../stores/useClubStore';
import { PanelAction } from './PanelAction';
import { PickerOverlay } from './PickerOverlay';
import { ProfileNameDialog } from './ProfileNameDialog';
import { clubSections } from './pickerSections';

interface Props {
  selectedId: string;
  onSelect: (id: string) => void;
  onClose: () => void;
}

interface ClubDraft {
  id?: string;
  name: string;
  base_type: string;
  loft_deg: number;
}

export function ClubPicker({ selectedId, onSelect, onClose }: Props) {
  const { t } = useI18n();
  const { clubs, lofts, loaded, useMyClubs, setUseMyClubs } = useClubStore();
  const [draft, setDraft] = useState<ClubDraft | null>(null);
  const [editingName, setEditingName] = useState(false);
  const [name, setName] = useState('');
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const scrollRef = useRef<HTMLDivElement>(null);
  const drag = useDragScroll(scrollRef);

  const edit = (club?: CustomClub) => {
    const base = club?.base_type ?? (ALL_CLUBS.some((item) => item.id === selectedId) ? selectedId : 'driver');
    setDraft({ id: club?.id, name: club?.name ?? '', base_type: base, loft_deg: club?.loft_deg ?? lofts[base] });
    setError('');
  };

  const save = async (remove = false) => {
    if (!draft || busy) return;
    setBusy(true);
    setError('');
    try {
      if (remove && draft.id) await socketService.removeCustomClub(draft.id);
      else await socketService.saveCustomClub(draft);
      setDraft(null);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Could not save club. Try again.');
    } finally {
      setBusy(false);
    }
  };

  return (
    <>
      <PickerOverlay
        title={draft ? (draft.id ? 'Edit club' : 'Add club') : t('app.selectClub')}
        selectedId={selectedId}
        sections={useMyClubs || draft ? [] : clubSections()}
        onSelect={onSelect}
        onClose={() => {
          if (!busy) {
            if (draft) setDraft(null);
            else onClose();
          }
        }}
        toolbar={
          !draft ? (
            <div className="custom-clubs__toolbar">
              <label>
                <input type="checkbox" checked={useMyClubs} onChange={(event) => setUseMyClubs(event.target.checked)} />{' '}
                Use my clubs
              </label>
              {useMyClubs ? (
                <PanelAction disabled={!loaded} onClick={() => edit()}>
                  Add club
                </PanelAction>
              ) : null}
            </div>
          ) : undefined
        }
      >
        {draft ? (
          <form
            className="custom-clubs__form"
            onSubmit={(event) => {
              event.preventDefault();
              void save();
            }}
          >
            <label>
              Name
              <button
                type="button"
                className="custom-clubs__name"
                onClick={() => {
                  setName(draft.name);
                  setEditingName(true);
                }}
              >
                {draft.name || 'Enter club name'}
              </button>
            </label>
            <label>
              Type
              <select
                value={draft.base_type}
                onChange={(event) =>
                  setDraft({ ...draft, base_type: event.target.value, loft_deg: lofts[event.target.value] })
                }
              >
                {ALL_CLUBS.map((club) => (
                  <option key={club.id} value={club.id}>
                    {club.name}
                  </option>
                ))}
              </select>
            </label>
            <label>
              Loft (°)
              <div className="custom-clubs__loft">
                <button
                  type="button"
                  aria-label="Decrease loft"
                  onClick={() => setDraft({ ...draft, loft_deg: Math.max(1, draft.loft_deg - 0.5) })}
                >
                  −
                </button>
                <input
                  aria-label="Loft (°)"
                  type="number"
                  min="1"
                  max="90"
                  step="0.5"
                  value={Number.isFinite(draft.loft_deg) ? draft.loft_deg : ''}
                  onChange={(event) => setDraft({ ...draft, loft_deg: event.target.valueAsNumber })}
                />
                <button
                  type="button"
                  aria-label="Increase loft"
                  onClick={() => setDraft({ ...draft, loft_deg: Math.min(90, draft.loft_deg + 0.5) })}
                >
                  +
                </button>
              </div>
            </label>
            <p className="custom-clubs__hint">Loft is descriptive. Estimates use the selected club type.</p>
            {error ? <p role="alert">{error}</p> : null}
            <div className="custom-clubs__actions">
              <PanelAction type="submit" disabled={busy || !draft.name.trim() || !Number.isFinite(draft.loft_deg)}>
                Save
              </PanelAction>
              <PanelAction variant="secondary" disabled={busy} onClick={() => setDraft(null)}>
                Cancel
              </PanelAction>
              {draft.id ? (
                <PanelAction variant="secondary" disabled={busy} onClick={() => void save(true)}>
                  Delete club
                </PanelAction>
              ) : null}
            </div>
          </form>
        ) : useMyClubs ? (
          <div className="custom-clubs__list" ref={scrollRef} {...drag}>
            {!loaded ? <p>Loading clubs…</p> : !clubs.some((club) => club.enabled) ? <p>No custom clubs yet.</p> : null}
            {clubs
              .filter((club) => club.enabled)
              .map((club) => (
                <div className="custom-clubs__row" key={club.id}>
                  <button
                    type="button"
                    className="custom-clubs__select"
                    aria-pressed={club.id === selectedId}
                    onClick={() => onSelect(club.id)}
                  >
                    <strong>{club.name}</strong>
                    <span>
                      {getClubName(club.base_type)} · {club.loft_deg}°
                    </span>
                  </button>
                  <PanelAction variant="secondary" aria-label={`Edit ${club.name}`} onClick={() => edit(club)}>
                    Edit
                  </PanelAction>
                </div>
              ))}
          </div>
        ) : undefined}
      </PickerOverlay>
      {editingName && draft ? (
        <ProfileNameDialog
          mode="add"
          titleOverride="Club name"
          name={name}
          onChange={setName}
          onConfirm={() => {
            setDraft({ ...draft, name: name.trim() });
            setEditingName(false);
          }}
          onCancel={() => setEditingName(false)}
        />
      ) : null}
    </>
  );
}
