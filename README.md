# InverterController
circuit to control MicroSolar inverter using remote cable socket
My vacation is your worst nightmare. Well, at least that is what the bumper sticker says – it’s referring to Burning Man. It’s well known that among the tens of thousands of people attending this arts festival in Nevada at the end of each Summer there are lots of high tech luminaries. I also have gone many times – not to say that I am a luminary. It’s a common tradition of so-called ‘burners’ to build complex projects to take with them that provide some of the comforts of home and more during the week of survival camping in the Black Rock Desert. Ad hoc showers, aluminum can recycling stations, elaborate cooking arrangements, alternative energy systems, coffee carts, colorful lighting setup, fire shooting vehicles, huge sound systems – pretty much anything you can think of and many you never even imagined – are brought out and assembled on the “playa.”

There is nothing for sale there – you must plan on bringing everything you need. There might be some sociable gifting, and people will probably help you out in a pinch, but start with the basics, like 2 gallons of water per person per day, and go from there. Notwithstanding, you can buy coffee and ice at what they call Center Camp. That’s it. Everything else is on you. The ice is for people’s coolers. And we are talking about 80,000 people camping in the desert heat. Of course, people bring RV’s (some consider that cheating) and others have generators and refrigerators (might also be cheating). I have relied on ice and coolers for many of my ten day stints out there. Needless to say, dealing with ice can be a major hassle. Getting very cold and heavy ice at Center Camp and bringing it back to your camp on foot or bicycle can be an ordeal. When it melts, it fills your cooler with slushy water that inevitably finds its way into your food. 

So I decided, what better project than to use solar power to run a small freezer to freeze blue ice packs? With a freezer working well enough, I can swap between two sets of blue ice packs each day. One set is getting deep frozen while the other set is keeping my Yeti cooler nice and cold. Last Summer, I started to assemble my system. It needed to be as compact as possible, weather proof, not easily damaged (either on the trip there or once set up), self-sustaining, reliable and reasonably priced. 

I ruled out an off the shelf solution because I wanted to better understand the realities of using solar power for something that we usually count on running off the mains. So, I started researching and shopping. The starting point for the whole system is the freezer. This established the requirements for everything else. The good news is that a 1.1 cubic foot freezer requires around 60 watts when running. This seemed like a totally doable load. 

Even though this is a solar powered system, a solar panel cannot deliver enough instantaneous power to turn over a motor. Therefore a battery would be needed. The first tests I performed were to start up the small 1.1 cubic foot Igloo freezer that I had bought as a factory second, on ebay for about $100. My first test using a 100 Watt combo inverter battery pack accomplished absolutely nothing. I then sought out a power meter to measure the wattage that was needed by the freezer. 

Plugging the unit into the wall with a watt meter showed load of well over 500 watts for up to 10 seconds at start up. This moved my requirement from a 100 watt inverter to a 600-1000 watt inverter. The other problem I was facing is that sealed lead acid batteries are not the best for driving high loads. I happen to own a large lithium power pack (battery and inverter combo) that can deliver 1000 watts which had no problem starting the freezer. But a stand-alone pure sine wave 1000 watt inverter can cost hundreds of dollars. I was sure that a 400 to 600 watt inverter could be made to work. Incidentally the large battery pack was not an option for the freezer because it can only handle 100 watts of solar power as input for charging. It would be a challenge to run the freezer and recharge its 1200 watt hours with the remaining 30-40 watts if needed. 

Because of the low weight and high current ratings of lithium batteries I started a search for an economical lithium battery pack. However, it had to be compatible with the common lead-acid chemistry charge controllers available on Amazon. I abandoned a couple of really heavy sealed lead acid batteries and bought one, then another, Dakota Lithium batteries - http://a.co/5Tty2HL
These have lots of surge power and are each rated at 120 watt hours. They are small, light and well priced.

I had a 700 watt modified sine wave inverter, but there were several things I did not like about it. And what is more, it still could not reliably start the freezer. This kicked off a deep dive into refrigerator motor design. Small cheap freezers, and probably more expensive ones, are a study in minimalist design. Those small back spheres that hold the compressor are cranked out by the millions. Inside is a motor with two coils, compressor piston, and a sealed oil lubrication and cooling system. Basically, the oil is lifted and sprayed around the interior by grooves on the motor shaft. They also have overheat protection. 

The electronics for starting the motor are on the outside, usually in a small plastic case pushed over the three pins that connect to the internals. In my case the power switch energizes the ‘run’ coil and the ‘start’ coil is initially connected but is disconnected when the current running through a small thermal switch causes it to open. Brute force, that assumes unlimited starting current and uses no intelligence. The worst-case scenario is when the compressor has been running and is then shut off and restarted again right away. The start coil is not engaged and the stalled motor draws hundreds of watts until the over-temperature inside the compressor triggers, which cannot be good for the motor. 

More sophisticated and better built appliances use inductors and/or capacitors in conjunction with relays and sensing circuits to deliver the starting coil current. This decreases the intial current draw substantially. I discovered an after-market so-called hard-start device designed for the standard refrigerator compressor motor. It is made by Supco - http://a.co/3ubL5wI It comes as a one pierce sealed unit that connects to pins that the plastic housing was connected to. While I do not have detailed numbers, it is clear that this $30 device solved most of the starting current problem I was facing. The freezer would start reliably and I was certain that an even lower powered inverter would work.

The 700 watt inverter was a modified sine wave inverter, which causes power loss in inductive loads like motors. I was looking build the most power efficient system,  and this seemed like something to improve. Also, a side effect of the modified sine waveform is that the motor runs hotter – due to the wasted power being transformed to heat. This particular inverter had a fan that would run even at low loads. It seemed unacceptable to have a 6-8 watt inverter load at idle. 10% of the system power would be going into the inverter at a minimum.

I wanted to set up a timer so that the freezer only ran during daylight hours. There were two choices. I could switch the AC power with a light timer, which are cheap and easy to setup. This has the disadvantage of requiring uninterrupted AC power to keep time properly. Another option is a DC timer, which has the advantage of not running the inverter at idle (3-5W) for the 18 hours the freezer is not running. If I have 240 watt hours of battery, you can see that ~40 watts hours adds up to a large percentage to lose every night. 

The system must be designed so the batteries have enough power reserve to start the motor in the morning, so current leakage overnight to run the idle inverter is unacceptable. A few low sun days could run this system into the ground. I found the perfect DC timer - http://a.co/c0d2RDR - that uses a battery to keep time, so it is immune to power outages. It is capable of switching large loads – the system can pull upwards of 10 Amps at startup. 

Back on Amazon I found a cheap pure sine inverter that looked like it would meet my needs. It is a MicroSolar 600W Pure Sine Inverter – http://a.co/cbzLs62 - for $89. Why not! It arrived and it worked great. There is no fan running in the inverter except under heavy loads – e.g. for start up, not at normal running currents. Soon though I discovered the Achilles heel of this system. 

I started out thinking that I could get by with 100 watts of solar panels. The cruel truth about solar panels is that they often output well below their rated power. The worst offenders are the flexible panels – like the ones I wanted to use because they weigh less, are harder to damage and take up less room in transit. After many experiments, I opted for 4 panels each rated at 120 watts for $120 each. https://www.solarblvd.com/product_info.php?products_id=3082 They can output close to 80-90 watts each in full overhead sun. So, the whole array can output an impressive 350 watts peak power. I could probably pare down to fewer panels, but the extra generation capability means faster battery recovery and more tolerance for cloudy days. It also means I can charge other loads and run the freezer at the same time. Think batteries for lighting, etc. 

The most important link in the system is the charge controller. This takes the raw DC power from the solar panel array and feeds it to the 12V batteries. The panels each output between 12 and 25 volts. If they are wired in series this typically means there is 80V DC feeding into the controller. Solar panels are very fussy about the IV curve they produce. For any given lighting situation and load there is an optimal IV point which has the panel producing the maximum power. 

There are two kinds of charge controllers: PWM and MPPT. PWM is pulse width modulation, and MPPT is maximum power point tracking. You want MPPT. PWM just toggles the current to battery at varying duty cycles to charge the battery. PWM is not very efficient and it does not allow the panels to produce the maximum power. MPPT uses a microprocessor that walks the charging voltage up and monitors the current to calculate the change in total power. MPPT will bring the voltage up to the point where the maximum power is produced by the panel. Then there is circuitry that produces the desired output voltage for the batteries. This extra circuitry is more expensive, but the efficiencies are often in the high 90% range. There is a substantial price boost for MPPT, but for the system efficiency requirements it seemed worth it. I chose a MPPT unit made by Renogy - http://a.co/1GFnZSI The alternative was buying more batteries or solar panels. 

The inverter draws so much power at the freezer startup it exceeded the “load” terminal rating on the charge controller, so the inverter load just gets connected to the battery terminals. In essence the charge controller thinks it is charging the batteries when the load is powered. The batteries are acting as a ballast. The one requirement for the charge controller I purchased is that it must always have 12V from the battery to run. If the battery runs down, it cannot recharge it, even if there is power coming from the solar panels. This is a reasonable restriction for a lead acid battery, but I am using lithium batteries with a low voltage cut out that is above the inverter low voltage cut out. The inverter will run the lithium batteries until they shut down. At which point the system needs a “jump start”. 

Once it became clear how serious this problem was, I realized I needed to monitor the voltage. Not only did it need to shut off above the minimum discharge level, to ensure the controller always had power, it needed to wait until the batteries had recovered significant charge before re-engaging the inverter and freezer load. 

Another major problem was that because the on-off control for the inverter was designed using a momentary switch, the unit would need manual attention every time the DC power to it was restored. There was no way to leave the unit ‘on’ so it would power on after DC voltage was applied when the timer turned on in the morning.

However, my choice of inverter turned out to be fortuitous. It came with a wired remote on-off switch; but it was even better because it also featured ‘run’ and ‘fault’ indicator lights. This combination meant that I could design a circuit that plugged in to the socket for the remote switch module and use a microcontroller to actively manage the system behavior. I could design a circuit to monitor the battery voltage and turn the inverter on and off as needed to keep the battery levels exactly where I wanted them. I could probably even eliminate the timer, but there is some advantage to shutting everything down well after dark.

I have extensive experience building projects with Arduinos – small easy to program and use microcontroller circuits. It’s easy to buy these small prebuilt boards that can be programmed over USB from a PC. They come with an on-board voltage regulator chip that operates with 12 to 15 volts, so the battery for the solar system can provide the power. Internally Arduinos operate on 5V, which is the voltage regulator output. The Arduino comes with digital I/O pins and analog input pins connected to the internal ADC. There is also support for SPI and I2C for connecting external devices and peripherals. 

I needed to bring a 12V line high or low to create the effect of pressing the button on the inverter remote control, and also read three analog signals – run, fault and the battery line voltage. The ADC on the Arduino only supports 0-5V, so a voltage divider is required. This can be built with a 1M ohm and 100K ohm resistor. I reverse engineered the signal lines on the remote cable. It has 4 wires. One is always ‘high’ and the run, fault and start-button all use it as the return. When the run or fault light is on, their voltage goes low, creating a current flow through the LED. The start button is pulled high when the button is pushed, because then it is connected to the high signal.

To control the start button I designed a simple circuit with an NFET and a pull up resistor on the drain side. With this the drain floats high until the NFET is closed by a 5V gate voltage. Then it is pulled down to ground. I own a milling machine, called the OtherMill, that makes it easy to produce my own custom printed circuit boards. I created a circuit in EagleCAD and then designed a PCB layout that can be milled on the OtherMill. It has pads for piggy back connections to the Arduino board I chose to use – Arduino Nano. These cost about $5 each and can be purchased on Amazon or Ebay.

Next came the software development. The Arduino comes with a free development environment. There are many libraries that make reading the analog signals and controlling the digital I/O’s easy. I also found coding example for the voltage divider. With some trial and error, I was able to write a program that would start up the inverter when power was applied and then monitor the status of the inverter and the battery voltage. I set a low voltage cutoff of 12 volts and have it wait until the battery voltage reaches 13.4 to restart, ensuring that the batteries have a reasonable level of charge before resuming operation.

My circuit also needed to avoid the death spiral of turning the unit back on after it had just stopped running. As is mentioned above if the freezer attempts to restart after just being turned off, the stalled motor draws heavy current and overheats. I added code to check to see if there is a fault and then ensures that there is a two minute delay before attempting a restart. 

Having the automatic control was nice, but running without any external display or indicators was difficult during software debug. There is a capability to add println() debugging when the Arduino is plugged into a PC, but I wanted to know the system status when the circuit is running in the field and not attached to a PC. This was solved with a 4 line LCD display that is compatible with the Arduino and costs about $15. I added code to display the battery voltage, system status and whether a fault had occurred on the inverter. The LCD display runs on the I2C bus and required a slight redesign of the PCB to add a 4 pin connector.

The last piece of this project is to add a small box for the custom electronics. I already purchased a larger waterproof case to hold the all the electronics: inverter, charge controller, timer, custom circuit and batteries. I opted to design a small circuit box using 3D CAD software and use a 3D printer to fabricate it. My favorite 3D modeling software is Onshape. A free account allows ‘public’ designs and a limited number of ‘private’ designs. The user interface is all web based, but don’t be fooled. This is a high-end tool, that uses cloud compute resources to provide sophisticated functionality. I own a FlashForge Creator Pro 3D printer and printed the box first in PLA and then in ABS. Hopefully the finished design for the box will hold up in the withering heat at Burning Man. 

Everything fits into the waterproof case and there is a water resistant flap for inserting cables from the solar array and the inverter. There is an additional load line to recharge another external 12V battery pack for other uses. There will be some clean up and rewiring to make thing more tidy. However, the whole system is up and running in my back yard. I also ran a series of tests to see how cold the cooler can stay with swapping cold packs every evening. I am looking forward to testing everything out in the Spring on a relaxing car camping trip. It will be nice to have cold beer without having to make ice runs. 
