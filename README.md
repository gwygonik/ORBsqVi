# ORBsq Vi

ORBsq Vi is an algorithmic CV generator. It is based on an original hardware module called Orbweaver, but later rebranded as ORBsq [(with MIDI and GB01 versions also made)](https://gwygonik.github.io/Wygonium-Info/).

<img src="https://github.com/gwygonik/WygoniumModules/blob/main/site/ORBsqVi_UI.png" width="350" alt="ORBsq Vi main interface in its default state" />

At its core, ORBsq Vi generates stepped sequences using an orbital path around a configurable point (**Base**) in a 2D noise field. The size of the orbit can be modified to be very small, with minimal changes in noise, to very large with large changes in the noise using the (**Range**) parameter.

Additionally, the orbit can be set to drift slightly on its circular path (**Drift**); moving the slider above the centerline will drift forward, while below centerline while reverse the drift direction. There are three different drift patterns, selectable via a toggle switch below the **Drift** slider. These are **Flat**, **Hemisphere**, and **Sphere**. Flat moves all the steps together, while Hemisphere and Sphere introduce sinusoidal offsets between PI and 2*PI. Finally, the speed of the drifting can be adjusted with the knob below the drift type.

While **Base** and **Range** determine the position in, and variance of the noise, the **Amp** parameter determines the output voltage amplitude of the noise range. The voltage range can be selected to be ±5V, 0-10V, or 0-5V. This allows flexibility in how the output is used, whether as pitch, modulation, or other destinations. Below the voltage output range is a button to allow the step voltages to be inverted. 

The last slider is a **Filter** module. This will allow for filtering steps based on either an algorithmic sequence (**ALG**) based on the current noise field, or a Euclidean sequence (**EUC**) with adjustable **Offset**.

Each of these parameters can be controlled via a 0-10v CV input, which overrides the slider setting.

Since the path through the noise is a circular orbit, that means it starts and ends near the same point, making the sequences loop nicely. As ORBsq Vi is algorithmic, not random, the sequences are also repeatable for any given set of paramaters, so you can even sequence them to create verse/chorus-type segments.

To keep things musically relevant, the orbit is broken down into a definable number of steps, from 2 to 16 (red **Steps** knob). Since the entire orbit is divided into steps, this does not simply truncate or expand the sequence, but instead gives more or less detail between the start and end.

ORBsq Vi will step through the sequence when a trigger is sensed on the **Trig In** input, and will automatically loop back to the first step after the last. The **Reset In** input will reset the sequence to the first step upon trigger.

ORBsq Vi provides three CV outputs, each with their own trigger output. **Main**, **Filter**, and **Drone**. **Main** is any step that is not filtered. **Filter** is any step that has been filtered. **Drone** is, basically, just the first step CV with trigger.

<!--
Context menu items:
- **"Drift Main Steps"** will enable/disable drifting of non-filtered steps.
- **"Drift Filtered Steps"** will enable/disable drifting of filtered steps.
- **"Drift Drone"** will enable/disable drifting of the drone step.
- **"Reset also Resets Drift"** will reset the drift state when a trigger is received on the **Reset** input.
-->

## Video demos (YouTube):

[![ORBsq Vi demo video](https://github.com/gwygonik/WygoniumModules/blob/main/site/vidcover.jpg)](https://youtu.be/m9-blrdRVsM)

## CPU Usage Note

Step values are only generated when **Base** or **Range** are adjusted and can be rather CPU intensive (up to 10% @ 44.1k samplerate). All other parameters, including **Drift** and **Filter**, only augment the generated steps, therefore have no impact to CPU. The average CPU usage during non-core parameter editing is < 1% @ 44.1k samplerate. Therefore, say you have an external CV source like a LFO continually adjusting the **Range** parameter, you can expect to see higher CPU usage than with just occassional changes. (These percentages are based on using ORBsq Vi in VCV Rack 2 on a 2015 MacBook Pro, so YMMV though probably for the better)

## Additional license info

The OpenSimplex2 noise code utilized in this module was released as public domain via the Unlicense License. Besides in this repo, you can find the source here: (https://gist.github.com/Markyparky56/e0fd43e847ac53068603130df3e8e560)

# Changelog

<!--
## 2.0.4
- Guard against crash on Windows with no audio interface

## 2.0.3
- Context menu options added! (See docs above)
- Minor visual tweaks
-->

## 2.0.2
- Filter offset no longer regenerates noise field
- Filter offset also stays at 0 instead of sometimes jumping up to 1
- Re-worked Filter parameter slider to better select 0 (middle) value and outer edges

## 2.0.1
- Initial release
