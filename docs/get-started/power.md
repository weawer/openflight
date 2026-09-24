# Powering OpenFlight

Everything about getting power into the unit: which route to pick, what to buy,
and how to wire it without destroying the boards. The [parts list](parts.md)
covers everything else.

For living with the UPS once it is built — Pi OS configuration, the battery
gauge, troubleshooting — see the
[Geekworm X1202/X1206 operator guide](../build/battery.md).

> [!CAUTION]
> **If your route involves a DC barrel plug, read
> [Barrel-jack polarity](#barrel-jack-polarity) before you wire or plug in
> anything.** A 5.5 × 2.1 mm barrel plug has no keying. A centre-negative
> supply mates perfectly with a centre-positive jack and reverses the rail
> into the Pi, the UPS and both radars at once. There is no fuse and no
> protection diode in the way. This is the single most expensive mistake
> available in this build, and the only routes that cannot make it are
> [Route 1](#route-1-the-official-27-w-usb-c-supply) and
> [Route 2b](#route-2b-the-same-ups-on-a-captive-usb-c-supply), which
> have no barrel plug in them at all.

## What the Pi 5 actually needs

**5.1 V at 5 A.** The current matters more than it looks. A Pi 5 only releases
its full downstream USB power budget when the supply tells it that 5 A is
available. A normal USB-C PD charger tops out at 3 A at 5 V, the Pi then caps
its USB ports, and the radars brown out or fail to enumerate. Every route below
exists to deliver a genuine 5 V 5 A to the Pi.

The load is real: about 25.5 W (5.1 V × 5 A) with both radars on the USB budget
and the screen lit.

## Choose a route

| Route | Unplug at the case? | Batteries | Polarity risk | Buy |
|---|---|---|---|---|
| **1. [Official 27 W USB-C supply](#route-1-the-official-27-w-usb-c-supply)** | **No** — the cable is captive to the brick and threads in through a rear opening | No | **None** | One supply |
| **2. [Geekworm X1202 / X1206 UPS HAT](#route-2-the-geekworm-x1202-x1206-ups-hat)** | Yes, DC barrel | Yes | **Yes** | UPS, four cells, DC adapter, panel jack, leads, button |
| **2b. [The same UPS on a captive supply](#route-2b-the-same-ups-on-a-captive-usb-c-supply)** | **No** — nothing detaches; the brick's cable is fixed | Yes | **None** | UPS, four cells, button, and Route 1's supply |
| **3. [DC in, USB-C out, no batteries](#route-3-dc-in-usb-c-out-no-batteries)** | Yes, DC barrel | No | **Yes** | Geekworm `Pi5-5V5APD`, a ≥30 W supply, panel jack |

Routes 2 and 3 end in a DC barrel plug, so they carry the polarity warning.
**Route 2b is the exception, and it is a trade rather than a free win**: it
runs the same UPS off Route 1's captive supply, so you get batteries with no
barrel plug anywhere in the build, but nothing detaches at the case.

### Then: what plugs into the barrel jack

Routes 2 and 3 put a DC socket in the back of the case. A wall adapter is the
obvious thing to plug into it and the one the parts tables assume, but it is
not the only one. **Neither of the two below is a way to power the unit on its
own** — each only changes what feeds that socket, with the UPS or the
converter behind it unchanged.

| Feeding the DC input | What it is | Why you would | ~Price |
|---|---|---|---|
| **Wall adapter** — the default | A 12-18 V supply, centre positive | Simplest, and it is what [Sizing the adapter](#sizing-the-adapter) covers | ~$15 |
| **[A USB-C PD charger or power bank](#add-on-a-usb-c-pd-charger-or-power-bank)** | A VFLEX or a PD trigger converts one into a barrel plug | Run from the chargers and power banks you already own, including away from a wall | $8 |
| **[Power over Ethernet](#add-on-power-over-ethernet)** | An 802.3bt splitter turns the run into 12 V on a barrel plug | One cable carries network and power | ~$35 |

---

## Route 1: the official 27 W USB-C supply

The default, and the one to pick unless you specifically want something the
others give you.

| Part | Description | Link | ~Price |
|------|-------------|------|--------|
| **27 W USB-C Power Supply** | Official Pi 5 supply, 5.1 V 5 A. **It must be this supply, or one that genuinely negotiates 5 V at 5 A over USB PD.** Plug it straight into the Pi: routing it through a USB-C extension or a panel-mount pass-through adds contact resistance, causes voltage sag, and can make the 5 V 5 A negotiation fail — and a panel-mount USB-C pass-through rated for 5 A is hard to find in the first place. Not needed if you power the Pi from a UPS HAT | [Adafruit](https://www.adafruit.com/product/5814) | $14 |

### The cable is fixed, and it stays captive to the case

The cable is part of the supply, not an accessory: Raspberry Pi's own
[27 W product brief](https://datasheets.raspberrypi.com/power-supply/27w-usb-c-power-supply-product-brief.pdf)
lists it as a specification — "Cable: 1.2 m 17AWG, white or black" — and the
mechanical drawings show it leaving the brick with no connector at that end.
Raspberry Pi uses the word **captive** for the 15 W supply, which is built the
same way. You cannot unplug the cable from the brick.

That matters here because **there is no panel connector to unplug at either**,
and no way to add one: as the row above says, a panel-mount USB-C pass-through
carrying a genuine 5 A is not a practical part to buy, and putting one in the
run risks the 5 V 5 A negotiation even if you find it. So the cable itself
comes in through one of the
[openflight-enclosure v3](https://github.com/open-flight/openflight-enclosure)
shell's rear openings — it fits, and the next section is the measurement that
says so — and the supply stays permanently tethered to the unit. To carry the rig
somewhere you coil the brick and its 1.2 m of cable and take the whole thing.

### Getting a USB-C plug through the wall

**Both captive routes rest on this** — this one and
[Route 2b](#route-2b-the-same-ups-on-a-captive-usb-c-supply) below, which feeds
the same supply to a UPS instead of to the Pi. Neither has a USB-C panel
connector in the run, so in both the plug itself has to pass through one of the
[rear openings](https://github.com/open-flight/openflight-enclosure/blob/main/docs/parts/shell.md#rear-io-openings).

The official supply's plug measures **12.0 mm across at its widest**, so it
goes through the Ø12.5 mm DC hole, and through the 16.0 × 14.0 mm Ethernet
cut-out with room to spare if you are not fitting the coupler. Another brick's
plug may be fatter, so measure that one before you rely on it.

> [!NOTE]
> **Dry-fit before the boards go in.** Whichever board takes the plug, its
> USB-C socket faces the rear recess, and the enclosure repo documents the
> openings rather than the clearance above that board edge. On Route 2b that
> edge is the more crowded of the two, because the UPS's USB-C socket sits
> alongside its own DC jack. Once the case is closed, the plug is inside it.

### Why that is still the right default

Accept the tether and you get the shortest, safest build in this document:

- **Nothing to wire.** No panel jack, no splices, no leads to cut.
- **No polarity to get wrong.** USB-C is keyed and the handshake is
  negotiated. There is no way to reverse a rail with it. Every other route
  puts an unkeyed barrel plug in your hand.
- **Nothing extra to buy** beyond the supply itself.
- **One failure mode fewer.** No battery chemistry, no charge controller, no
  wide-input converter.

If you do not already know why you want a UPS or a detachable lead, stop here.

---

## Route 2: the Geekworm X1202 / X1206 UPS HAT

Batteries plus mains, and a DC barrel input you can unplug at the case. This is
the route that needs the most parts and the most care.

> [!IMPORTANT]
> **Buying a UPS HAT is not one purchase.** The board arrives with neither
> cells nor a way to get power into it. Budget for all of it up front:
>
> 1. the **HAT** itself;
> 2. **four matching cells** — not included, and they must be the right type
>    (see the rows below);
> 3. a **DC power adapter** in the board's voltage window, with enough current;
> 4. a **panel-mount DC jack with flying leads** for the case's Ø12.5 mm rear
>    hole, so the supply is detachable;
> 5. **the lead that joins that jack to the board.** The short version is a
>    **screw-terminal barrel plug**: strip the panel jack's flying leads, screw
>    them into the block, plug it into the board's own barrel jack. Nothing
>    else. The alternative is a JST XH lead into the board's `XH2.54-2P`
>    header, which needs two Wago 221 splices to join it to the panel jack;
> 6. a **12 mm momentary push button** and its leads, if you want the rear
>    power button to work.
>
> Items 4 and 5 are the ones people forget. Without them the adapter has
> nowhere to plug in once the case is closed.

### The boards

| Part | Description | Link | ~Price |
|------|-------------|------|--------|
| **Geekworm X1202 UPS HAT** | Rechargeable Pi 5 power from four **unprotected, flat-top 18650** cells (Geekworm's wiki is explicit on both: max 18.5 mm diameter, 65.3 mm length, and *"Do not use 18650 battery with built-in protection circuit"*). Cells are not included. Input is **6-18 V DC at 3 A or more** on the 5.5 × 2.1 mm jack, **or** 5 V 5 A on its USB-C, which Geekworm lists as *"Compatible with Raspberry Pi USB-C Power Supply"* — and their spec table says in bold **"Never Use Both at the Same Time"**. Delivers up to 5.5 A, so it can fast-charge at 3 A while running the Pi | [Geekworm](https://geekworm.com/products/x1202) / [wiki](https://wiki.geekworm.com/X1202) / [Amazon](https://www.amazon.com/dp/B0CRZ4ZXQW) | ~$48 + cells |
| **Geekworm X1206 UPS HAT** | Larger option: four **unprotected 21700** cells, advertised to 20,000 mAh. Holders are on the board. Cells are not included. Output 5.1 V ±5 %, max 6 A. Same XH2.54 power-button header as the X1202. **Check the board revision before choosing a supply — see the warning below** | [Geekworm](https://geekworm.com/products/x1206) / [wiki](https://wiki.geekworm.com/X1206) | $52 + cells |

> [!WARNING]
> **X1206 V1.1 and V2.0 take completely different power.** Geekworm's own wiki
> puts it bluntly: *"Check the version number on the board and use the correct
> power supply, or the board may burn out."*
>
> | Revision | Accepts |
> |---|---|
> | **V1.1** | **USB-C 5 V (≥5 A)**, which Geekworm *"strongly recommend"* over its DC input; that input takes only **5-6 V at ≥5 A** on the DC5521 or XH2.54. Wide voltage is **not** supported — do **not** put 12 V into it |
> | **V2.0** | 9-18 V DC (12 V 3 A recommended), **or** USB-C 5 V ≥5 A |
>
> Geekworm updated the X1206 to V2.0 on 7 April 2026, so a board bought since
> then should be V2.0 — but read the silkscreen rather than assuming. The
> 12 V DC route in this guide is **V2.0 only**. The X1202 has no such split: it
> takes 6-18 V across the board.

### Route 2b: the same UPS on a captive USB-C supply

**The barrel jack above stays the default for a UPS build.** A supply you can
unplug at the case is most of the reason to put a panel jack in the shell, and
everything in this route — the adapter, the jack, the wire sizing, the
polarity rules — is what to follow if you want that.

This is the alternative **if you can live with a supply that does not detach**.
The official 27 W brick is captive at both ends of the problem: the cable will
not come off the brick, and once the plug is inside a closed shell it will not
come off the board either. Accept that and the whole DC run disappears.

It works because every board above takes **5 V at 5 A on its own USB-C
socket**, and 5 V at 5 A is exactly what the
[Route 1](#route-1-the-official-27-w-usb-c-supply) supply exists to make. The
UPS still hands the Pi its 5.1 V 5 A from the cells; all that changes is how
power reaches the UPS.

This is not a hack around the vendor. Geekworm list the X1202's USB-C input as
*"Compatible with Raspberry Pi USB-C Power Supply"*, and on the **X1206 V1.1**
they go further: *"We strongly recommend using the USB C 5 V (≥5 A) port for
power instead"* of its DC jack. On that revision this is the recommended input,
not a fallback.

The cable goes into the **UPS**, never the Pi. Geekworm again: *"Supply power
through the X1202, not the Raspberry Pi's USB-C port. Choose either the X1202
USB-C input or the DC input; never use both at the same time."*

**What you give up.** Two things, and the first is the one to be sure about
before you order anything:

- **Nothing detaches at the case.** The supply and the unit travel as one
  piece, and getting the lead off the board means opening the shell. If you
  move the rig between a bay and a garage, stow it in a bag, or want the option
  of swapping supplies, build the barrel-jack route above instead — that is
  exactly what it is for.
- **Charging is slower, and under a heavy load it stops.** 5 V × 5 A is 25 W
  in, and the Pi alone can pull 25.5 W with both radars on its USB budget, so
  at peak there is nothing left over to charge with — the pack will even give
  up a little to cover the gap while you are plugged in. It makes that back
  whenever the rig is idle or off. Geekworm's *"3 A fast charging while
  powering the system"* is a property of the **DC** input, which runs at
  12-18 V and has the headroom for both jobs at once. If you play for hours on
  mains and want a full pack at the end of it, use the DC route.

**What you skip in exchange.** The DC adapter, the panel jack and the lead
into the board all drop out — about $25 of parts, and every step in this page
where a polarity can be got wrong. There is no barrel plug in this build, so
[Barrel-jack polarity](#barrel-jack-polarity) does not apply to it. You keep
Route 1's $14 supply rather than replacing it, so in money the saving is only
about $11; the three fewer parts and the missing failure mode are the real
return. You still want the **button** if you want the rear power button to
work — that is a separate header and a separate hole, unaffected.

The plug reaches the board through the wall exactly as it does on Route 1 —
the measurement and the dry-fit warning are in
[Getting a USB-C plug through the wall](#getting-a-usb-c-plug-through-the-wall).

### Getting power to it

| Part | What it is | Link | ~Price |
|------|-----------|------|--------|
| **DC adapter, 5.5 × 2.1 mm barrel, centre positive** | The wall supply. **12-18 V at 3 A or more** covers everything; see [Sizing the adapter](#sizing-the-adapter). Examples: MEAN WELL GST36 (12 V 3 A) or Geekworm's own PSU60 (12 V 5 A) | [Mouser (EU plug)](https://www.mouser.com/c/?q=GST36E12-P1J) / [Mouser (US plug)](https://www.mouser.com/c/?q=GST36U12-P1J) / [Amazon (PSU60)](https://www.amazon.com/dp/B0BDF89DCB) | ~$15 |
| **Panel DC jack with leads, 5.5 × 2.1 mm** | The socket for the case's Ø12.5 mm rear hole, so the supply is detachable. Buy one **pre-wired with flying leads**, so the DC run itself needs no soldering. See [Which panel jack](#which-panel-jack) | [Mouser (Tensility 10-03609)](https://www.mouser.com/c/?q=10-03609) / [Tensility](https://www.tensility.com/products/10-03609) / [Amazon (6-set)](https://www.amazon.com/dp/B0DP6MNQQB) | $7-10 |
| **Wago 221-412 lever connectors, ×2** | **Only if you need to join two leads** — the header option, or a barrel plug that came pre-wired. Tool-free lever splices, one per conductor. A screw-terminal plug needs none, because the panel jack's leads go straight into it | [Mouser](https://www.mouser.com/c/?q=WAGO%20221-412) / [Amazon (bag of 10)](https://www.amazon.com/dp/B072PT3JNL) | $1 |
| **UPS end: barrel plug or XH lead** | **Simplest is a 5.5 × 2.1 mm screw-terminal plug** such as the Adafruit 369: the panel jack's leads screw straight into it and it goes into the board's own jack, so there are no Wagos and no XH lead in the build at all. Otherwise a JST XH 2-pin lead for the `XH2.54-2P` header, or a pre-wired plug on a lead heavy enough for 3 A. See [Two ways in](#two-ways-into-the-ups) | [Adafruit 369](https://www.adafruit.com/product/369) / [Mouser (4872)](https://www.mouser.com/ProductDetail/Adafruit/4872?qs=sGAEpiMZZMsvnOgGvSjZeHfx0dldyM%2FtbKuuneru8OfHveSm083OQA%3D%3D) | $1-2 |
| **Power button: 12 mm momentary + its lead** | A **12 mm** panel-mount **momentary** push button with 0.11" quick-connect tabs, plus one pair from an Adafruit 1152 pack. That pair is the entire connection: two 0.11" quick-connects pre-crimped on one end, a 2-pin JST on 2.5 mm / 0.1" spacing on the other, 200 mm of 22 AWG, ten pairs to a pack. Quick-connects onto the button, JST plug into the board's `PSW` header — **no Wagos**, nothing soldered or crimped | [Mouser (1152)](https://www.mouser.com/ProductDetail/Adafruit/1152?qs=GURawfaeGuAkPRIbdozo3A%3D%3D) | ~$6 |

Cable runs for all of these are measured in
[Cable lengths](parts.md#cable-lengths-enclosure-v3).

#### Sizing the adapter

Geekworm's requirement is a **current**, not a wattage: 3 A or more. The same
3 A buys very different things at different voltages, so check the amps against
the volts rather than reading "6-18 V" as "any adapter".

| Input | Total | What it does |
|---|---|---|
| **12 V 3 A** | 36 W | Runs the Pi at full load **and** charges at full rate. Geekworm's own adapters are 12 V |
| 9 V 3 A | 27 W | Runs the Pi, but charging slows under load |
| 6 V 3 A (X1202 only) | 18 W | Cannot carry a full Pi load; the cells drain while plugged in |

The arithmetic behind it: the Pi 5 can draw 25.5 W with both radars on its USB
budget, and charging adds up to about 12 W when the cells are low, plus
converter losses. A little over 25 W runs the Pi; it does not also charge it.
**12-18 V at 3 A or more** covers both.

#### Which panel jack

The **Tensility 10-03609** is the Mouser line: an overmoulded jack on a 305 mm
18 AWG lead, rated 7.5 A, M11 × 1.0 thread with nut and lock washer, for panels
1.5-4.5 mm thick — the shell face is 3.0 mm, so it clamps. Its Ø11 thread sits
1.5 mm loose in the Ø12.5 mm hole and its Ø13.5 mm head has flats at 9.8 mm,
so a sliver of the hole shows at the flats.

On Amazon, pre-wired **DC-099 style kits with a 12 mm thread** fit the hole as
drawn: a [6-set](https://www.amazon.com/dp/B0DP6MNQQB) with 150 mm 20 AWG
leads, a [10-pack rated 5 A](https://www.amazon.com/dp/B08F26JJKM) with 150 mm
18 AWG, or the [DaierTek set](https://www.amazon.com/dp/B0BD46CP5Y), which also
includes pre-wired plugs for the barrel option below.

Not this one: Tensility's other 2.1 mm lead, **10-02878**, has a Ø10.8 mm
thread but only a Ø12.5 mm flange — the same as the hole, so it has nothing to
clamp against.

#### Make sure the wire can carry the current

Easy to overlook, because the DC run is the only part of this build that
carries real current. The UPS's DC input can pull **3 A or more**, and thin
signal wire in that run gets warm, drops voltage, and at worst softens its own
insulation inside a closed plastic box.

- **Use 20 AWG or heavier** for both conductors of the DC run. The panel jacks
  above already ship with 18-20 AWG leads, so buying the right jack solves most
  of this.
- **22 AWG is the floor**, and only because the run is short — under 100 mm
  from the rear hole to the UPS. The Adafruit 1152 lead pair is 22 AWG and is
  acceptable on that basis.
- **Do not use 26 AWG.** That is why Adafruit 4872 is not the pick for the XH
  lead even though its connector is right: the wire is sized for signals, not
  for 3 A.
- **Check the jack's own rating too.** Tensility's 10-03609 is rated 7.5 A and
  the Amazon 10-pack 5 A, so both clear 3 A with room. The Wago 221-412 takes
  24-12 AWG and is not the limit.
- **Keep it short, and do not coil the slack.** Extra length is extra voltage
  drop, and a coil of current-carrying wire in a sealed case is a heat source.

If you lengthen any of this for a different enclosure, size the wire for the
supply you actually plug in, not for the 3 A minimum.

#### Two ways into the UPS

Pick one. The barrel option is fewer parts and fewer joints, so default to it
unless you have a reason not to.

**Barrel option.** Panel jack leads → a 5.5 × 2.1 mm **screw-terminal barrel
plug**, such as the **Adafruit 369** → the UPS's own barrel jack. Strip the
panel jack's flying leads, screw them into the block, plug it in. That is the
whole run: no Wago splices, no XH lead, nothing crimped or soldered, and the
block is marked **+** and **−**, which makes the polarity check harder to get
wrong. Adafruit note that those labels *"assume a positive-tip
configuration"*, which is what every input on this page wants. They do not
publish a wire range for the block, so if your panel jack came with thick
18 AWG leads, check they seat before you count on this route — the 20 AWG
leads on the Amazon DC-099 kits are the easier fit.

> A plug that comes **pre-wired** does the same job, but it arrives with its
> own lead, so you are back to joining two leads with the same two Wago 221
> splices the header option needs. If you go that way, check the lead carries
> 3 A ([wire sizes](#make-sure-the-wire-can-carry-the-current)) and buzz out
> which conductor reaches the tip before you splice — its colours are no more
> trustworthy than the panel jack's. The
> [DaierTek set](https://www.amazon.com/dp/B0BD46CP5Y) in the panel-jack list
> ships panel jacks and pre-wired plugs in one box.

**Header option.** Panel jack leads → two Wago 221 splices → a JST XH 2-pin
lead → the UPS's `XH2.54-2P` DC input. One pair from the Adafruit 1152 pack in
the button row is exactly that lead (XH plug, 200 mm of 22 AWG, quick-connects
cut off), so the pack covers both jobs. Adafruit 4872 is a matching pair but
its 26 AWG wire is thin for the 3 A this input can draw.

**The UPS itself has no screw terminals.** Geekworm give it three power
inputs and no terminal block: the `XH2.54-2P` header, its own 5.5 × 2.1 mm
barrel jack, and the USB-C socket. Any screw terminal in this build is on the
**plug you buy**, not on the board — the Adafruit 369 is a barrel plug with a
terminal block on its tail, which is exactly why the panel jack's leads can
land in it directly.

Those three inputs are alternatives, not a sequence: use one. The USB-C socket
is covered by the same rule — see
[Keeping the official supply](#route-2b-the-same-ups-on-a-captive-usb-c-supply),
which needs neither the panel jack nor anything else in this section.

Which panel jack physically fits is a property of the case, not of the UPS.
The **openflight-enclosure repository documents the rear I/O hole sizes** for
the current shell — the Ø12.5 mm DC hole, the Ø12.5 mm button hole and the
Ethernet cut-out, and the 3.0 mm panel thickness a threaded jack has to clamp:
see [Rear I/O openings](https://github.com/open-flight/openflight-enclosure/blob/main/docs/parts/shell.md#rear-io-openings).
Check a candidate jack against those numbers before ordering, and re-check them
if you are printing a different or older shell.

#### The button has to be momentary, and 12 mm

The X1202/X1206 expose their external power button on an XH2.54 2-pin header
and need a **momentary** (spring-back) switch: the board reads how long the
button is held, the way the Pi 5's own button works, so a latching or toggle
switch will not do.

Size it to the case. Both round rear holes in the v3 shell are **Ø12.5 mm**, so
you want a **12 mm** panel-mount momentary button with 0.11" (2.8 mm)
quick-connect tabs, which is what the Adafruit 1152 lead pair pushes onto.
Adafruit showed the 1152 pack out of stock when checked; Mouser's stock is
unverified.

**The button run needs no Wagos and no separate XH lead.** One 1152 pair
already spans the whole distance: Adafruit's own description is *"two 0.11"
quick-connects pre-crimped onto 20cm long wires … then terminated together in
a JST 2.5mm/0.1" spaced 2-pin connector"*. Push the quick-connects onto the
button's tabs, plug the JST end into the board's `PSW` header, and there is
nothing to join in the middle. The Wagos in the parts table are for the **DC**
run, and only for its header option, where the same kind of lead is used with
its quick-connects cut off and the bare ends spliced to the panel jack.

> [!NOTE]
> Adafruit call that connector a *"JST 2.5mm/0.1" spaced 2-pin"* rather than
> naming the series. XH is the 2.5 mm JST family and Geekworm label the `PSW`
> header `XH2.54-2P`, so they mate — but that is the one detail to confirm
> with the seller if you want certainty before ordering.

---

## Route 3: DC in, USB-C out, no batteries

You want to unplug the supply at the case, but you do not want lithium cells,
the charging rules, or the cost. A wide-input DC-to-USB-C module does that: a
DC barrel jack in the case's rear hole feeds the module, and the module hands
the Pi a **detachable** USB-C cable at a genuine 5 V 5 A.

| Part | Description | Link | ~Price |
|------|-------------|------|--------|
| **Geekworm `Pi5-5V5APD` dual PD power module** | 44 × 55 mm board that takes a wide DC input and outputs USB-C **5 V 5 A** with a real PD CC signal, so the Pi sees a 5 A supply and keeps its full USB budget. Sold in two input variants: **9-24 V on a 5.5 × 2.1 mm DC jack**, or 9-24 V on a 3.81 mm 2-pin terminal block. It can also be fed from USB-C PD, which it negotiates at 12 V. Stable 5 A, 6 A peak; a jumper cap raises the output 0.2 V. Ships with a small fan, removable below 3 A. "Dual" is two things at once: dual **inputs** (USB-C PD or DC jack) and dual **outputs** (USB-C and USB-A — note the USB-A port carries no charging protocol) | [Geekworm](https://geekworm.com/products/rpi5-5v5a-pd) / [wiki](https://wiki.geekworm.com/Pi5-5V5APD) | ~$20 |

> [!WARNING]
> **This module needs more than 30 W in.** Geekworm states *"Total input power
> must be ＞ 30 W"*, and their own FAQ answers the obvious question: fed from
> the official **27 W** Raspberry Pi supply the module tops out around 4.5 A
> and shuts down at 4.6 A. So a 27 W brick will not do. Use a 12 V supply of
> 36 W or more on the DC jack, or a 45 W-plus USB-C PD charger that offers 12 V.

You still need the **panel DC jack** and the **DC adapter** from Route 2 — the
module sits inside the case and the jack is what makes the supply detachable.
You do not need the cells, the lead into a UPS board, or the button.

> [!NOTE]
> **Fit is not solved for you.** The v3 case has mounts for the Pi, the UPS
> boards and the Adafruit bays, but no dedicated mount for this 44 × 55 mm
> module, and the no-UPS build normally uses the
> [x1202 Pi adapter](https://github.com/open-flight/openflight-enclosure/blob/main/docs/parts/adapters.md)
> plate. Plan where it goes before ordering.

---

## Add-on: a USB-C PD charger or power bank

**Not a way to power the unit on its own.** This is an add-on to a Route 2 or
Route 3 build: it changes what you plug into the barrel jack in the back of the
case, and the UPS or the converter behind that jack is unchanged.

The reason to want it is that you already own USB-C chargers and PD power
banks. A Pi 5 cannot take those directly at 5 A, but a **PD sink** converts one
into the DC voltage the UPS or the `Pi5-5V5APD` wants, ending in the same
5.5 × 2.1 mm centre-positive plug a wall adapter would. That includes using it
with the UPS, if you want batteries *and* the option of running from a power
bank.

| Part | Description | Link | ~Price |
|------|-------------|------|--------|
| **VFLEX Base** (Werewolf) | The easy option. A USB-C Power Delivery **sink**: plug it into any PD charger or power bank and it outputs the voltage you configured, 5 V to 48 V at up to 5 A. You set that once from [vflex.app](https://vflex.app) and it is stored on the device, so there is no switch or solder blob to knock out of place later. Its **Type B tip is 5.5 × 2.1 mm, centre positive** — exactly the barrel the X1202, X1206 V2.0 and `Pi5-5V5APD` take | [Werewolf](https://werewolf.us/products/vflex-base) / [datasheet](https://werewolf.us/vflex/base/datasheet) / [manual](https://werewolf.us/vflex/user-manual) | $8 |
| **Generic USB-C PD trigger board** | The cheaper, blunter alternative, also sold as a "PD decoy": it selects a fixed 5/9/12/15/20 V with a DIP switch, a button or a solder jumper, and some ship as a finished USB-C-to-barrel cable | [what one is](https://learn.adafruit.com/usb-pd-hacks/things-to-know) | $5-10 |

VFLEX covers the full PD range — SPR fixed, PPS, SPR AVS, EPR fixed and EPR AVS
— and every output tip Werewolf sells is centre positive. Prefer it to a
generic trigger unless cost is decisive: on a trigger board the voltage is set
by hardware you can knock into the wrong position, and a mis-set trigger
feeding a UPS is an expensive afternoon.

### Choosing the voltage and the charger

Two things have to line up, and a PD charger will silently refuse if they do not:

1. **A voltage your charger actually offers.** USB-C PD fixed steps are 5, 9,
   12, 15 and 20 V, and **12 V is optional** — plenty of good chargers skip it
   and offer 9, 15 and 20 V instead. Check the PDO list printed on the charger.
2. **A voltage your board accepts.** X1202: 6-18 V. X1206 **V2.0**: 9-18 V.
   `Pi5-5V5APD`: 9-24 V. So **9 V, 12 V or 15 V** all work for every one of
   them; 20 V is too high for the UPS boards.

Then check the current at that voltage. The UPS wants **3 A or more**, so 9 V
needs 27 W, 12 V needs 36 W and 15 V needs 45 W from the source. The
`Pi5-5V5APD` wants **more than 30 W** whatever the voltage. A 45 W or 65 W PD
charger or power bank set to **15 V** is the comfortable pick; 12 V if your
source offers it.

> [!NOTE]
> A power bank has to sustain that for as long as you play. Check its
> **continuous** PD rating, not the peak number on the box, and remember that
> feeding a UPS from a power bank charges the UPS's cells from the bank's
> cells, which is lossy. If portable running is the goal, the UPS's own
> batteries are the better answer and the power bank is the top-up.

---

## Add-on: Power over Ethernet

**Also not a route on its own**, and the same shape as the one above: a PoE
splitter turns the 48 V on the Ethernet run into **12 V on a 5.5 × 2.1 mm
barrel plug**, which is exactly what Route 2's UPS and Route 3's converter
already accept. It replaces the wall adapter, not the board behind it.

The reason to want it is one cable for network and power. It is the tidiest
option where the rig has structured cabling to sit on, and the v3 case is
already arranged for it: the shell's Ethernet opening sits next to the DC hole,
so the two leads go in side by side.

| Part | What it is | Link | ~Price |
|------|-----------|------|--------|
| **802.3bt PoE splitter, 12 V DC out** | Splits the Ethernet run into data and power. You want **12 V on a 5.5 × 2.1 mm plug**, which sits inside the X1202's 6-18 V window, the X1206 V2.0's 9-18 V and the `Pi5-5V5APD`'s 9-24 V. Example: the REVODATA PS5712BG, 802.3bt, **12 V 3 A (36 W)**, with 2.5 Gbps passthrough and isolation, short-circuit and overvoltage protection. Price moves with the region, so read it off the listing | [Amazon UK](https://www.amazon.co.uk/dp/B0F1F8JX4G) | ~$35 |

### The source matters as much as the splitter

A splitter can only pass on what the injector or switch at the far end gives
it, and the three PoE standards are far apart:

| Standard | Delivered to the device | Enough here? |
|---|---|---|
| 802.3af | 12.95 W | **No.** Not even the Pi on its own |
| 802.3at (PoE+) | 25.5 W | Marginal. Runs the Pi with nothing spare |
| **802.3bt (PoE++)** | 51 W and up | **Yes.** What this splitter needs for its full 36 W |

REVODATA say the same thing about their own part: feed it from 802.3at and it
does not reach 36 W, and an 802.3bt source is what unlocks the full output.

Then budget it like any other supply. The Pi 5 draws up to 25.5 W with both
radars on its USB budget, so 36 W runs it with roughly 10 W spare. On Route 2
that spare is what charges the cells, so charging is slower under full load
than it would be from a 60 W adapter. On Route 3, where there are no cells,
36 W clears the `Pi5-5V5APD`'s "more than 30 W" requirement with room.

### Wiring it into the case

The splitter lives **outside** the case, which is what the two rear openings
are for. The PoE run from the wall goes into the splitter. A short patch lead
goes from the splitter into the case's Ethernet coupler. The splitter's barrel
plug goes into the case's DC jack. Nothing changes inside: the DC jack reaches
the UPS exactly as in [Two ways into the UPS](#two-ways-into-the-ups).

> [!WARNING]
> **Meter the splitter's plug before it goes near a board.** PoE splitters are
> conventionally centre positive, but the listing does not state it and the
> plug does not enforce it. This is the same unkeyed 5.5 × 2.1 mm barrel as
> everywhere else here, so treat it the same way and read
> [Barrel-jack polarity](#barrel-jack-polarity) first.

---

## Barrel-jack polarity

> [!CAUTION]
> **Get this wrong and you destroy the Pi, the UPS and both radars at once.**
>
> A 5.5 × 2.1 mm barrel plug is **not keyed and not polarised**. A
> centre-negative supply pushes into a centre-positive jack with a satisfying
> click and reverses the rail into everything downstream. Nothing in this build
> is protected against it.

**Every DC input in this guide is centre positive.** Centre pin (the tip) is
**+**, outer sleeve is **−**.

- **Geekworm X1202 and X1206** — centre pin positive. Raised by JedS on
  [openflight#273](https://github.com/open-flight/openflight/pull/273#issuecomment-5779406632),
  who checked the X1202 and then asked Geekworm directly about the X1206:
  *"I got a response from Geekworm. The center pin of the DC 5521 jack is
  positive (+) also for X1206."* Geekworm's current wiki pages do not state the
  polarity either way, so treat that exchange, not the wiki, as the source.
- **VFLEX** — every tip cable Werewolf sells is centre positive, Type B
  (5.5 × 2.1 mm) included.
- **Geekworm `Pi5-5V5APD`** — not documented by Geekworm. Meter it.

### Rules

1. **Read the supply's own symbol.** Every DC adapter carries the
   centre-positive/centre-negative pictogram near the ratings. It is a small
   circle with a line from the middle and a line from the outside, marked
   **+** and **−**. The middle must be **+**.
2. **Meter it before it goes anywhere near a board.** Power the adapter with
   nothing attached, put the multimeter's black probe on the outer sleeve and
   the red probe on the inside of the barrel, and read **positive** volts. A
   negative reading means centre negative — do not use it.
3. **Do not trust wire colour on a panel jack.** Pre-wired panel jacks are not
   consistent: red is usually the centre pin and black the sleeve, but it is
   not guaranteed. **Buzz it out.** Put the meter on continuity, plug a spare
   barrel plug into the jack, and find which lead reaches the plug's tip. That
   lead is **+**.
4. **Wire the jack to match.** The lead that goes to the jack's **centre**
   contact is **+**, and it goes to **+** on the UPS's XH input or into the
   **+** screw of the barrel-plug terminal block. The `+` is marked on the
   board and on the Adafruit 369 block.
5. **Check the voltage before the board goes in.** With the jack wired and the
   case open, plug in the adapter and meter the far end of the leads. Right
   voltage, right sign, then connect the board.
6. **Never feed two inputs at once.** Geekworm's X1202 spec says it in bold:
   *"Never Use Both at the Same Time."* Barrel **or** USB-C, not both. And
   whichever you use, it goes into the **UPS board's** socket — never into the
   Pi's own USB-C port while the Pi is sitting on the UPS.

---

## Lithium cell safety

Only Route 2. Read this before the first charge.

> [!WARNING]
> **Never charge the cells below 0 °C (32 °F).** Lithium-ion cells charged
> below freezing plate metallic lithium onto the anode. That damage is
> permanent and it makes the cell unsafe, not merely weaker. Bring a cold rig
> indoors and let it warm up before connecting power.

Geekworm publishes one
[safety template](https://wiki.geekworm.com/Template:UPS_Safety_Warning)
that is carried on every one of their UPS product pages. The points that bite
in this build, in their words:

- *"Lithium Polymer and Li-ion batteries are volatile. Failure to read and
  follow the instructions below may result in fire, personal injury, and damage
  to property if charged or used improperly."*
- **"When charging the Battery Pack, please place the battery in a fireproof
  container. Do not leave the UPS shield on wood material or carpet
  unattended."** Note the scope: this is about **charging**, and the word is
  **fireproof**. Geekworm publishes no guidance about how to *store* cells, and
  does not specify a metal or airtight container — a sealed metal box is in
  fact the wrong shape for a venting cell. A purpose-made LiPo charging bag or
  a ceramic/steel tin left unsealed, on a hard non-combustible surface, is what
  this asks for.
- *"Never make a wrong polarity connection when charging or discharging battery
  packs."* The holders are marked; check every cell before the board is closed.
- *"Do not mix and use old batteries with new batteries, or batteries with
  different brand names."* Buy four identical cells at the same time.
- *"Please replace old batteries with new ones when they reach their service
  life or when they are two years old, whichever comes first."*
- *"Ensure your fingers do not touch the solder pads when inserting the battery
  into the battery holder, as this could cause a short circuit."*
- *"Make sure to insert the battery before turning on the UPS"* and before
  connecting the charger.
- The power adapter *"must [come] with overvoltage and surge voltage
  protection; otherwise, it may easily damage the circuit board."* Geekworm
  excludes damage from a substandard supply from warranty.

**Cell type is not a preference.** The X1202 takes four **unprotected,
flat-top 18650s**, max 18.5 mm diameter and 65.3 mm long. The X1206 takes four
**unprotected 21700s**. Both wikis say it plainly: *"Do not use [a] battery
with built-in protection circuit."* A protected cell is longer and its circuit
fights the UPS's own.

---

## Getting the DC route into the v3 case

The [openflight-enclosure v3](https://github.com/open-flight/openflight-enclosure)
shell has three rear openings on one sloped face, and
[their dimensions are documented in that repo](https://github.com/open-flight/openflight-enclosure/blob/main/docs/parts/shell.md#rear-io-openings):
a 16.0 × 14.0 mm Ethernet cut-out, a **Ø12.5 mm DC hole**, and a **Ø12.5 mm
button hole**. The face is **3.0 mm** thick, which is the panel thickness your
jack and button have to clamp.

**None of the three is a USB-C panel connector**, and that is not an oversight
in the shell: a pass-through rated for a genuine 5 A is not a practical part to
buy, and the repo's own USB-C rear shells are retired, marked **EOL** for
*"USB-C spec compatibility"*. It is why Route 1's supply stays captive and why
the DC routes exist at all. A USB-C *plug* still passes through one of these
openings — see
[Getting a USB-C plug through the wall](#getting-a-usb-c-plug-through-the-wall).

From the panel jack, two ways to reach the UPS — pick one, not both:

**Barrel option, the shorter one.** Jack leads → the screw terminals of a
5.5 × 2.1 mm plug such as the Adafruit 369 → the UPS's own jack. Nothing else
in the run, and the block is marked **+** and **−**.

**Header option.** Jack leads → two Wago 221 splices → a JST XH 2-pin lead →
the UPS's `XH2.54-2P` DC input. No soldering on this run either.

Either way the run is short: about 45 mm straight and 80 mm routed from the
middle rear hole to the UPS's DC input, so any 150 mm jack lead reaches. Full
measurements are in [Cable lengths](parts.md#cable-lengths-enclosure-v3).

---

## What this costs

| Route | Added over a bare Pi | ~Price |
|---|---|---|
| 1. Official 27 W supply | The supply | $14 |
| 2. X1202 UPS | HAT $48 + four 18650s $24 + 12 V adapter $15 + panel jack $8 + the lead into the board $2-3 + button $6 | ~$104 |
| 2. X1206 UPS | the same list with a $52 HAT and four 21700s at $32 | ~$116 |
| 2b. Either UPS on a captive supply | The same HAT, cells and button, **minus** the adapter, the jack and the lead into the board, **plus** Route 1's $14 supply | ~$92 / ~$104 |
| 3. `Pi5-5V5APD` | Module $20 + 36 W supply $15 + panel jack $8 | ~$43 |
| Add-on: VFLEX | VFLEX Base + a PD source you already own, **on top of** route 2 or 3 | $8 |
| Add-on: PoE | An 802.3bt splitter + a PoE source you already own, **on top of** route 2 or 3 | ~$35 |

Cells are estimated at ~$6 each for 18650 (Samsung 35E, Molicel P28A, LG MJ1)
and ~$8 each for 21700 (Samsung 50E, Molicel P42A). Routes 2 and 3 include the
panel jack and leads, because without them the supply has nowhere to plug in
once the case is closed. Route 2 also **replaces** Route 1's $14 supply rather
than adding to it, so its net cost in a full build is $90 for the
X1202 and $102 for the X1206. Route 2b *keeps* that supply and uses it, which
is why the $14 sits inside its figure rather than being deducted from it. The
staged breakdown is in the [parts list](parts.md#cost-summary).
