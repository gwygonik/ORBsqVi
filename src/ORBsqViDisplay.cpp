#include <rack.hpp>
//#include <osdialog.h>
//#include <thread>

template <class TModule>
struct ORBsqViDisplay : rack::LedDisplay {
	TModule* module;
	rack::Vec displaySize;
	float ramp[16];
	int curstep = -1;
	int steps = 0;
	int filtersteps = 0;
	float curScale1 = 0.f;
	bool euclideanFilter = false;

	std::string fontPath = rack::asset::system("res/fonts/ShareTechMono-Regular.ttf");

	void drawLayer(const DrawArgs& args, int layer) override {

		if (layer == 1 && module) {
			for (int i=0;i<16;i++) {
				ramp[i] = module->displayStepVal[i];
				ramp[i] += std::sin(module->driftAcc + (i*module->drift_div)) * module->drift;
				ramp[i] *= module->curScale1;
				if (ramp[i] > 5.0f) ramp[i] = 5.0f - (ramp[i] - 5.0f);
				if (ramp[i] < -5.0f) ramp[i] = -5.0f + std::abs(ramp[i] + 5.0f);
			}

			curScale1 = module->curScale1;
			steps = module->steps;
			filtersteps = module->filter_steps;
			euclideanFilter = module->filterType < 0.5f;
			curstep = module->curStep;

			rack::Vec p;

			// Draw steps

			nvgScissor(args.vg, RECT_ARGS(args.clipBox));
			nvgBeginPath(args.vg);

			float stepX = (displaySize.x-2) / (float)steps;

			// bottom line
			p.x = rack::mm2px(1);
			p.y = rack::mm2px(displaySize.y-6);
			nvgMoveTo(args.vg, VEC_ARGS(p));
			p.x = rack::mm2px(displaySize.x - 1);
			nvgLineTo(args.vg, VEC_ARGS(p));
			nvgLineCap(args.vg, NVG_ROUND);
			nvgMiterLimit(args.vg, 2.f);
			nvgStrokeWidth(args.vg, 1.5f);
			nvgStrokeColor(args.vg, nvgRGB(0x80,0x80,0x80));
			nvgStroke(args.vg);

			// middle line
			nvgBeginPath(args.vg);
			p.x = rack::mm2px(1);
			p.y = rack::mm2px((2.0f+(displaySize.y-8.f)) / 2.f);
			nvgMoveTo(args.vg, VEC_ARGS(p));
			p.x = rack::mm2px(displaySize.x-1);
			nvgLineTo(args.vg, VEC_ARGS(p));

			nvgLineCap(args.vg, NVG_ROUND);
			nvgMiterLimit(args.vg, 2.f);
			nvgStrokeWidth(args.vg, 1.5f);
			nvgStrokeColor(args.vg, nvgRGB(0x30,0x30,0x30));
			nvgStroke(args.vg);

			// drone
			nvgBeginPath(args.vg);
			p.x = rack::mm2px(1);
			p.y = rack::mm2px(clamp(rescale(ramp[0], -5.f, 5.f, displaySize.y-8.f, 2.f),2.f,displaySize.y-8.f));
			nvgMoveTo(args.vg, VEC_ARGS(p));
			p.x = rack::mm2px(displaySize.x-1);
			nvgLineTo(args.vg, VEC_ARGS(p));
			nvgLineCap(args.vg, NVG_ROUND);
			nvgMiterLimit(args.vg, 2.f);
			nvgStrokeWidth(args.vg, 4.f);
			nvgStrokeColor(args.vg, nvgRGBA(0x10,0xf0,0xd0,0x40));
			nvgStroke(args.vg);
		
			// steps

			for (int i=0;i<steps;i++) {
				if (module) {
					if (module->curSeqState[i] == true) {
						nvgBeginPath(args.vg);
						p.x = rack::mm2px(1 + (i*stepX+2));
						p.y = rack::mm2px(clamp(rescale(ramp[i], -5.f, 5.f, displaySize.y-8.f, 2.f),2.f,displaySize.y-8.f));
						nvgMoveTo(args.vg, VEC_ARGS(p));
						p.x = rack::mm2px(1 + ((i+1)*stepX)-1);
						nvgLineTo(args.vg, VEC_ARGS(p));
						nvgLineCap(args.vg, NVG_BUTT);
						nvgMiterLimit(args.vg, 2.f);
						nvgStrokeWidth(args.vg, 3.f);
						nvgStrokeColor(args.vg, nvgRGB(0xd0,0xd0,0xd0));
						nvgStroke(args.vg);
					} else {
						nvgBeginPath(args.vg);
						p.x = rack::mm2px(1 + (i*stepX+2));
						p.y = rack::mm2px(clamp(rescale(ramp[i], -5.f, 5.f, displaySize.y-8.f, 2.f),2.f,displaySize.y-8.f));
						nvgMoveTo(args.vg, VEC_ARGS(p));
						p.x = rack::mm2px(1 + ((i+1)*stepX)-1);
						nvgLineTo(args.vg, VEC_ARGS(p));
						nvgLineCap(args.vg, NVG_BUTT);
						nvgMiterLimit(args.vg, 2.f);
						nvgStrokeWidth(args.vg, 1.0f);
						nvgStrokeColor(args.vg, nvgRGB(0xd0,0xd0,0xd0));
						nvgStroke(args.vg);
                    }
				}
			}

			// beat indicator

			if (curstep >= 0) {
				nvgBeginPath(args.vg);

				p.x = rack::mm2px(1 + ((curstep)*stepX));
				p.y = rack::mm2px(displaySize.y-7);//box.size.y-1;
				nvgMoveTo(args.vg, VEC_ARGS(p));
				p.x = rack::mm2px(1 + ((curstep+1)*stepX));
				nvgLineTo(args.vg, VEC_ARGS(p));

				nvgLineCap(args.vg, NVG_BUTT);
				nvgMiterLimit(args.vg, 2.f);
				nvgStrokeWidth(args.vg, 3.f);
				nvgStrokeColor(args.vg, rack::SCHEME_WHITE);
				nvgStroke(args.vg);
			}

			std::shared_ptr<rack::Font> font = APP->window->loadFont(fontPath);
			if (font) {
				nvgFontSize(args.vg, 12);
				nvgFontFaceId(args.vg, font->handle);
				nvgFillColor(args.vg, nvgRGB(0xd0,0xd0,0xd0));
				std::string stepsStr = "Steps:" + std::to_string(steps);//"STEPS:";// std::to_string(steps);
				nvgTextAlign(args.vg, NVG_ALIGN_LEFT);
				nvgText(args.vg, rack::mm2px(1.5),rack::mm2px(37.5), stepsStr.c_str(), NULL);

				if (euclideanFilter) {
					std::string stepsStr = "EuFlt:" + std::to_string(filtersteps);//"STEPS:";// std::to_string(steps);
					nvgTextAlign(args.vg, NVG_ALIGN_RIGHT);
					nvgText(args.vg, rack::mm2px(displaySize.x-1),rack::mm2px(37.5), stepsStr.c_str(), NULL);
				}
			}
		}


		nvgResetScissor(args.vg);
		Widget::drawLayer(args, layer);
	}


};
