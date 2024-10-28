#include "plugin.hpp"
#include "OpenSimplexNoise.hpp"
#include "ORBsqViDisplay.cpp"

struct ORBsqVi : Module {

	enum ParamId {
		FILTER_PARAM,
		STEPS_PARAM,
		POSITION_PARAM,
		VARIANCE_PARAM,
		DRIFT_PARAM,
		AMP_PARAM,
		DRIFTSPEED_PARAM,
		OFFSET1_PARAM,
		DRIFTTYPE_PARAM,
		VOLTSCALE_PARAM,
		FILTERTYPE_PARAM,
		INVERT_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		TRIGGER_INPUT,
		RESET_INPUT,
		POS_INPUT,
		VAR_INPUT,
		DRFT_INPUT,
		AMP_INPUT,
		DRIFTSPEED_INPUT,
		FILTER_INPUT,
		DRIFTTYPE_INPUT,
		VOLTSCALE_INPUT,
		FILTERTYPE_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		MAINCV_OUTPUT,
		MAINTRIG_OUTPUT,
		FILTERCV_OUTPUT,
		FILTERTRIG_OUTPUT,
		DRONECV_OUTPUT,
		DRONETRIG_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		INVERT_LIGHT,
		LIGHTS_LEN
    };

	int curStep, steps, lastSteps, filter_steps, seed;
	float base, variance, filter, filter2, drift;
	float curScale1, curOffset1;
	float curScale2, curOffset2;
	float lastPos, lastVar, lastFilter;
	float curSeqVal[16];
	bool curSeqState[16];
	float displayStepVal[16];
	float curVolt, droneVolt;
	float driftAmt, driftAcc, drift_div;
	bool triggerMain, triggerFiltered, triggerDrone;
	float TWO_PI = 2.f * M_PI;
	int delayCount = 0;
	float driftSpeed;
	bool invertVoltage = false;
	float filterType = 0.0f;
	float oldFilterType = 0.0f;
	float filterShift = 0.0f;
	float oldFilterShift = 0.0f;
	bool triggered = false;
	float baseDriftAcc = 0.00000125f;
	float currentDriftAcc = baseDriftAcc;
	bool canDriftNormal = true;
	bool canDriftFiltered = true;
	bool canDriftDrone = true;
	bool resetResetsDrift = false;

	dsp::SchmittTrigger inTrigger;
	dsp::SchmittTrigger inReset;
	dsp::PulseGenerator pulseOutputMain;  // main
	dsp::PulseGenerator pulseOutputFiltered; // filtered
	dsp::PulseGenerator pulseOutputDrone; // drone
	dsp::BooleanTrigger invertTrigger;

	OpenSimplexNoise simplexNoise;


	ORBsqVi() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(FILTER_PARAM, -1.f, 1.f, 0.f, "Filter");
		configParam(STEPS_PARAM, 2, 16, 8, "Number of Steps");
		configParam(POSITION_PARAM, 1.f, 10.f, 1.f, "Base Position");
		configParam(VARIANCE_PARAM, 1.f, 10.f, 1.f, "Range");
		configParam(DRIFT_PARAM, -1.f, 1.f, 0.f, "Drift Amount");
		configParam(AMP_PARAM, 0.f, 5.f, 0.f, "Amp");
		configInput(TRIGGER_INPUT, "Trigger");
		configInput(RESET_INPUT, "Reset");
		configInput(POS_INPUT, "Base 0-10v CV");
		configInput(VAR_INPUT, "Range 0-10v CV");
		configInput(DRFT_INPUT, "Drift 0-10v CV");
		configInput(AMP_INPUT, "Amp 0-10v CV");
		configInput(FILTER_INPUT, "Filter 0-10v CV");
		configParam(INVERT_PARAM, 0,1,1, "Invert Voltage Range");
		configOutput(MAINCV_OUTPUT, "Main Note CV");
		configOutput(MAINTRIG_OUTPUT, "Main Note Trig");
		configOutput(FILTERCV_OUTPUT, "Filtered Note CV");
		configOutput(FILTERTRIG_OUTPUT, "Filtered Note Trig");
		configOutput(DRONECV_OUTPUT, "Drone Note CV");
		configOutput(DRONETRIG_OUTPUT, "Drone Trig");
		configSwitch(DRIFTTYPE_PARAM, 0.f, 2.f, 0.f, "Drift Type", {"Sphere", "Hemisphere", "Flat"});
		configSwitch(VOLTSCALE_PARAM, 0.f, 2.f, 0.f, "Voltage Scale", {"-5V to +5V", "0V to +10V", "0V to +5V"});
		configSwitch(FILTERTYPE_PARAM, 0.f, 1.f, 0.f, "Filter Type", {"Euclidean", "Noise Algo"});
		configParam(OFFSET1_PARAM, 0, 16, 0, "Filter Offset (Euclidean Only)");
		configParam(DRIFTSPEED_PARAM, 1.0f, 10.0f, 1.f, "Drift Speed");

		paramQuantities[STEPS_PARAM]->snapEnabled = true;

		simplexNoise = OpenSimplexNoise(3518);

		curStep = -1;
		driftAcc = 0.0f;
		drift = 40.f;
		lastPos = -10.0f;
		lastVar = -10.0f;
		lastFilter = -10.0f;
		lastSteps = 30;
		seed = 1;

		for (int r=0;r<16;r++) {
			curSeqVal[r] = 0;
			curSeqState[r] = false;
			displayStepVal[r] = 0.0f;
		}

	}

	void onSampleRateChange(const SampleRateChangeEvent& e) override {
		currentDriftAcc = baseDriftAcc / (e.sampleRate / 44100.0f);
    }

	void onReset(const ResetEvent& e) override {
		Module::onReset(e);
		invertVoltage = false;
		canDriftNormal = true;
		canDriftFiltered = true;
		canDriftDrone = true;
		resetResetsDrift = false;
		curStep = -1;
		driftAcc = 0.0f;
	}

	void process(const ProcessArgs& args) override {
		bool dirty = false;

		steps = params[STEPS_PARAM].getValue();
		driftAcc += (currentDriftAcc * params[DRIFTSPEED_PARAM].getValue());
		if (driftAcc >= TWO_PI) driftAcc = 0.f;

		curScale1 = params[AMP_PARAM].getValue();
		if (invertTrigger.process(params[INVERT_PARAM].getValue() > 0.f)) {
			invertVoltage ^= true;
			dirty = true;
        }
	
		base = params[POSITION_PARAM].getValue();
		variance = std::pow(2,(float)params[VARIANCE_PARAM].getValue());
		drift = params[DRIFT_PARAM].getValue();
		filterType = params[FILTERTYPE_PARAM].getValue();
		filterShift = clamp(params[OFFSET1_PARAM].getValue(),0.f,(float)steps-1.f);

		if (inputs[POS_INPUT].isConnected()) {
			base = clamp(inputs[POS_INPUT].getVoltage(),1.f,10.f);
			params[POSITION_PARAM].setValue(base);
		}

		if (inputs[VAR_INPUT].isConnected()) {
			variance = std::pow(2,clamp(inputs[VAR_INPUT].getVoltage(),1.f,10.f));
			params[VARIANCE_PARAM].setValue(clamp(inputs[VAR_INPUT].getVoltage(),1.f,10.f));
		}

		if (inputs[DRFT_INPUT].isConnected()) {
			drift = clamp(rescale(inputs[DRFT_INPUT].getVoltage(),0.f,10.f,-1.f,1.f),-1.f,1.f);
			params[DRIFT_PARAM].setValue(drift);
		}

		if (inputs[AMP_INPUT].isConnected()) {
			curScale1 = clamp(rescale(inputs[AMP_INPUT].getVoltage(),0.f,10.f,0.f,5.f),0.f,5.f);
			params[AMP_PARAM].setValue(curScale1);
		}

		if (inputs[FILTER_INPUT].isConnected()) {
			filter = clamp(rescale(inputs[FILTER_INPUT].getVoltage(),0.f,10.f,-1.f,1.f),-1.f,1.f);
			params[FILTER_PARAM].setValue(filter);
		}


		filter = params[FILTER_PARAM].getValue();
		if ((filter > -0.02f) && (filter < 0.02f)) filter = 0.f;

		drift_div = 0.0f;
		if (params[DRIFTTYPE_PARAM].getValue() == 1.0f) {
			drift_div = M_PI/(float)steps;
		} else if (params[DRIFTTYPE_PARAM].getValue() == 0.0f) {
			drift_div = TWO_PI/(float)steps;
		}

		if ( (base != lastPos) || (variance != lastVar) || (steps != lastSteps) || dirty ) {
			// recalc ramps
			float cStep = TWO_PI / steps;
			float ang = 0.0f;
			float curVal = 0.0f;
			for (int r=0;r<steps;r++) {
				curVal = clamp(simplexNoise.Evaluate((float)base + std::sin(ang) * (variance/50.f), (float)base + std::cos(ang)*(variance/50.f), seed*10.f),-1.0f,1.0f);
				if (invertVoltage) curVal *= -1.0f;
				curSeqVal[r] = curVal;
				ang += cStep;
				displayStepVal[r] = curVal;
			}
			lastPos = base;
			lastVar = variance;
			lastSteps = steps;
			dirty = true;
		}

		if ((filter != lastFilter) || (filterType != oldFilterType) || (filterShift != oldFilterShift) || dirty) {
			if (filterType > 0.5f) {
				for (int r=0;r<steps;r++) {
					if (filter > 0) {
						if ((curSeqVal[r] <= filter) && (curSeqVal[r] >= filter*-1.0f+0.15f)) {
							curSeqState[r] = true;
						} else {
							curSeqState[r] = false;
						}
					} else if (filter < 0) {
						float tf = std::abs(filter);
						if ((curSeqVal[r] <= tf) && (curSeqVal[r] >= tf*-1.0f+0.15f)) {
							curSeqState[r] = false;
						} else {
							curSeqState[r] = true;
						}
					} else {
						curSeqState[r] = false;
					}

				}
			} else {
				// euclidean
				int current_filter = (int)floor(clamp(rescale(filter,-1.f,1.f,(steps*-1)-1,steps+1),(float)steps*-1,(float)steps));
				filter_steps = current_filter;
				if (current_filter == 0) {
					for (int i=0;i<steps;i++) {
						curSeqState[i] = false;
					}
				}
				if (current_filter > 0) {
					if (current_filter == steps) {
						for (int i=0;i<steps;i++) {
							curSeqState[i] = true;
						}
					} else {
						int num_pulses = steps - current_filter;
						// simple
						for (int i=0;i<steps;i++) {
							curSeqState[(i+(int)floor(filterShift))%steps] = !((((num_pulses * (i + 0)) % steps) + num_pulses) >= steps);
						}
					}
				} else {
					// negative euclidean
					if (abs(current_filter) == steps) {
						for (int i=0;i<steps;i++) {
							curSeqState[i] = false;
						}
					} else {
						int num_pulses = steps - abs(current_filter);
						// euclidian here since we have our own algo

						// simple
						for (int i=0;i<steps;i++) {
							curSeqState[(i+(int)floor(filterShift))%steps] = ((((num_pulses * (i + 0)) % steps) + num_pulses) >= steps);
						}
					}
				
				}
            }
			oldFilterType = filterType;
			oldFilterShift = filterShift;
			lastFilter = filter;
			dirty = false;
		}

		if (inReset.process(inputs[RESET_INPUT].getVoltage(), 0.01f, 2.f)) {
			curStep = -1;
			if (resetResetsDrift) {
				driftAcc = 0.0f;
            }
		}

		if (inTrigger.process(inputs[TRIGGER_INPUT].getVoltage(), 0.01f, 2.f)) {
			delayCount = 0;
			triggered = true;
		}

		if (triggered) {
			curStep++;
			curStep %= steps;

			curVolt = curSeqVal[curStep];
			droneVolt = curVolt;
			if (curSeqState[curStep]) {
				// normal unfiltered note
				if (canDriftNormal) {
					curVolt += std::sin(driftAcc + (curStep*drift_div)) * drift;
                }
            } else {
				// filtered note... drift filtered?
				if (canDriftFiltered) {
					curVolt += std::sin(driftAcc + (curStep*drift_div)) * drift;
                }
            }
			if (canDriftDrone) {
				droneVolt += std::sin(driftAcc + (curStep*drift_div)) * drift;
            }
			curVolt *= curScale1;
			droneVolt *= curScale1;

			if (curVolt > 5.f) {
				curVolt = 5.f - (curVolt - 5.f);
			}
			if (curVolt < -5.f) {
				curVolt = -5.f + (std::abs(curVolt) - 5.f);
			}
			if (droneVolt > 5.f) {
				droneVolt = 5.f - (droneVolt - 5.f);
			}
			if (droneVolt < -5.f) {
				droneVolt = -5.f + (std::abs(droneVolt) - 5.f);
			}



			if (params[VOLTSCALE_PARAM].getValue() == 2) {
				curVolt = rescale(curVolt, -5.f, 5.f, 0.f, 5.f);
				droneVolt = rescale(droneVolt, -5.f, 5.f, 0.f, 5.f);
			} else if (params[VOLTSCALE_PARAM].getValue() == 1) {
				curVolt = rescale(curVolt, -5.f, 5.f, 0.f, 10.f);
				droneVolt = rescale(droneVolt, -5.f, 5.f, 0.f, 10.f);
			}

			if (curSeqState[curStep]) {
				pulseOutputMain.trigger(1e-3f);
				outputs[MAINCV_OUTPUT].setVoltage(curVolt);
			} else {
				pulseOutputFiltered.trigger(1e-3f);
				outputs[FILTERCV_OUTPUT].setVoltage(curVolt);
			}
			if (curStep == 0) {
				pulseOutputDrone.trigger(1e-3f);
				outputs[DRONECV_OUTPUT].setVoltage(droneVolt);
			}

			triggered = false;


		}

		triggerMain = pulseOutputMain.process(args.sampleTime);
		outputs[MAINTRIG_OUTPUT].setVoltage(triggerMain ? 10.f : 0.f);

		triggerFiltered = pulseOutputFiltered.process(args.sampleTime);
		outputs[FILTERTRIG_OUTPUT].setVoltage(triggerFiltered ? 10.f : 0.f);

		triggerDrone = pulseOutputDrone.process(args.sampleTime);
		outputs[DRONETRIG_OUTPUT].setVoltage(triggerDrone ? 10.f : 0.f);

		lights[INVERT_LIGHT].setBrightness(invertVoltage ? 0.9f : 0.f);

	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();

		json_t* val = json_boolean(invertVoltage);
		json_object_set_new(rootJ, "invertVoltage", val);
		val = json_boolean(canDriftNormal);
		json_object_set_new(rootJ, "canDriftNormal", val);
		val = json_boolean(canDriftFiltered);
		json_object_set_new(rootJ, "canDriftFiltered", val);
		val = json_boolean(canDriftDrone);
		json_object_set_new(rootJ, "canDriftDrone", val);
		val = json_boolean(resetResetsDrift);
		json_object_set_new(rootJ, "resetResetsDrift", val);

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* val = json_object_get(rootJ, "invertVoltage");
		if (val) {
			invertVoltage = json_boolean_value(val);
		}
		val = json_object_get(rootJ, "canDriftNormal");
		if (val) {
			canDriftNormal = json_boolean_value(val);
		}
		val = json_object_get(rootJ, "canDriftFiltered");
		if (val) {
			canDriftFiltered = json_boolean_value(val);
		}
		val = json_object_get(rootJ, "canDriftDrone");
		if (val) {
			canDriftDrone = json_boolean_value(val);
		}
		val = json_object_get(rootJ, "resetResetsDrift");
		if (val) {
			resetResetsDrift = json_boolean_value(val);
		}
	}

};


struct ORBsqViWidget : ModuleWidget {
	ORBsqViWidget(ORBsqVi* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/ORBsqViPanel.svg")));

		ORBsqViDisplay<ORBsqVi>* display = createWidget<ORBsqViDisplay<ORBsqVi>>(mm2px(Vec(7.584, 10.842)));
		display->box.size = mm2px(Vec(66.113, 39.257));
		display->displaySize = Vec(66.113, 39.257);
		display->module = module;
		addChild(display);

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		// steps
		addParam(createParamCentered<Davies1900hRedKnob>(mm2px(Vec(18.811, 110.905)), module, ORBsqVi::STEPS_PARAM));

		// position (base)
		addParam(createParamCentered<LEDSliderGreen>(mm2px(Vec(11.535, 71.124)), module, ORBsqVi::POSITION_PARAM));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(11.535, 89.867)), module, ORBsqVi::POS_INPUT));

		// var (range)
		addParam(createParamCentered<LEDSliderGreen>(mm2px(Vec(26.087, 71.124)), module, ORBsqVi::VARIANCE_PARAM));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(26.087, 89.867)), module, ORBsqVi::VAR_INPUT));

		// drift
		addParam(createParamCentered<LEDSliderGreen>(mm2px(Vec(40.640, 71.124)), module, ORBsqVi::DRIFT_PARAM));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(40.640, 89.867)), module, ORBsqVi::DRFT_INPUT));
		// drift type
		addParam(createParamCentered<CKSSThree>(mm2px(Vec(40.640, 104.967)), module, ORBsqVi::DRIFTTYPE_PARAM));
		// drift speed
		addParam(createParamCentered<Trimpot>(mm2px(Vec(40.640, 119.445)), module, ORBsqVi::DRIFTSPEED_PARAM));

		// scale1 (amp)
		addParam(createParamCentered<LEDSliderGreen>(mm2px(Vec(55.192, 71.124)), module, ORBsqVi::AMP_PARAM));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(55.192, 89.867)), module, ORBsqVi::AMP_INPUT));
		// voltage scale type
		addParam(createParamCentered<CKSSThree>(mm2px(Vec(55.192, 104.967)), module, ORBsqVi::VOLTSCALE_PARAM));
		// voltage invert
		addParam(createParamCentered<LEDButton>(mm2px(Vec(55.192, 119.445)), module, ORBsqVi::INVERT_PARAM));
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(55.192, 119.445)), module, ORBsqVi::INVERT_LIGHT));

		// filter
		addParam(createParamCentered<LEDSliderGreen>(mm2px(Vec(69.744, 71.124)), module, ORBsqVi::FILTER_PARAM));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(69.744, 89.867)), module, ORBsqVi::FILTER_INPUT));
		// filter offset
		addParam(createParamCentered<Trimpot>(mm2px(Vec(69.744, 119.445)), module, ORBsqVi::OFFSET1_PARAM));
		// filter type
		addParam(createParamCentered<CKSS>(mm2px(Vec(69.744, 104.967)), module, ORBsqVi::FILTERTYPE_PARAM));

		// inputs
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(86.271, 19.423)), module, ORBsqVi::TRIGGER_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(86.271, 35.298)), module, ORBsqVi::RESET_INPUT));

		// outputs
		// main
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(86.271, 52.825)), module, ORBsqVi::MAINCV_OUTPUT)); //54.270836
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(86.271, 63.408)), module, ORBsqVi::MAINTRIG_OUTPUT));
		// filtered
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(86.271, 79.283)), module, ORBsqVi::FILTERCV_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(86.271, 89.867)), module, ORBsqVi::FILTERTRIG_OUTPUT));
		// drone
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(86.271, 105.742)), module, ORBsqVi::DRONECV_OUTPUT)); //54.270836
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(86.271, 116.325)), module, ORBsqVi::DRONETRIG_OUTPUT)); //54.270836

	}

	void appendContextMenu(Menu* menu) override {
		ORBsqVi* module = getModule<ORBsqVi>();
		menu->addChild(new MenuSeparator);
		menu->addChild(createMenuLabel("ORBsq Vi Options"));
		menu->addChild(createBoolPtrMenuItem("Drift Main Steps", "", &module->canDriftNormal));
		menu->addChild(createBoolPtrMenuItem("Drift Filtered Steps", "", &module->canDriftFiltered));
		menu->addChild(createBoolPtrMenuItem("Drift Drone", "", &module->canDriftDrone));
		menu->addChild(new MenuSeparator);
		menu->addChild(createBoolPtrMenuItem("Reset also resets Drift", "", &module->resetResetsDrift));
	}


};


Model* modelORBsqVi = createModel<ORBsqVi, ORBsqViWidget>("ORBsqVi");
