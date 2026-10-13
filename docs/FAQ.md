# FAQ

**Is this a real fly brain?**
No. It is a simulation of neurons wired according to a real fly's measured synapses. No living tissue is
involved, and the model leaves out much of real neurobiology (see [SCIENCE.md](SCIENCE.md)).

**Did anyone write flight code?**
The brain contains no flight code. Around it there is engineering: optic-flow computation, a linear
read-out from descending neurons, a safety governor, and the drone's own stabilising flight controller.
All of that is open in this repo.

**Why do the GIFs use 850 neurons and not 166,000?**
So anyone can run the demo in seconds without a 1.2 GB download. Two commands switch to MaleCNS.

**Does it learn?**
Not yet. The connectome weights are fixed. Dopamine-based plasticity (as in DOOMFLY) is on the roadmap.

**Can one brain fly a swarm?**
`flydrones swarm` copies one connectome into several brains with shared wiring and separate activity.
Each copy flies one drone. A real swarm would also need drone-to-drone collision avoidance.

**Windows?**
Yes. Use `.venv\Scripts\activate`, Tello over Wi-Fi works, Crazyradio needs the Zadig USB driver.
For the `serial` backend, use `--cam-backend dshow` for the capture card and pass the port by name
(`COM5`). If two cameras share an index, pass the DirectShow name instead: `--camera "USB Video"`.
`list_ports` sometimes refuses to enumerate inside a sandboxed shell - open the port directly instead.

**Can I use the drone's own camera as the fly's eyes?**
Yes, that is the `serial` backend. The downlink goes into a USB UVC capture device (640×480), the brain
reads it as `--camera`, and the three stick values go out on a serial port as 4 bytes at 20 Hz. See
[HARDWARE.md](HARDWARE.md#usb-camera-in-usb-serial-out-your-own-flight-controller).

**The drone does not respond, or responds to something random. What is wrong?**
Almost always the receiver, not the brain. Two classics: it is searching for `0xAA` as a delimiter
instead of decoding exactly 4 bytes per frame (`0xAA` is also a legal value, −86), and it has no
RX-loss failsafe so it holds the last command forever. Use `tools/uart_loopback_sim.py` as the
reference receiver and `tools/serial_monitor.py --decode` to see what is really on the wire.

**Can the brain run on the drone?**
MiniFly or a small sensorimotor core can run on a Raspberry Pi class computer. Full MaleCNS needs a
laptop-class CPU for now.

**How do I cite this?**
See [CITATION.cff](../CITATION.cff), and please cite the MaleCNS paper and Shiu et al. 2024.
