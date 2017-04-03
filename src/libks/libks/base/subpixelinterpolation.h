#ifndef KS_SUBPIXELINTERPOLATION_H
#define KS_SUBPIXELINTERPOLATION_H

namespace ks {
	// Common interpoltion schemes for subpixel accuracy
	class SubpixelInterpolation {
	public:
		enum InterpolationType {
			LINEAR,
			QUADRATIC
		};
	
		template <typename T>
		static float maxOffset(T c1, T c2, T c3, InterpolationType type) {
			T dl = c2 - c1;
			T dr = c2 - c3;
			
			// Handle special cases
			if(dl < 0)
				return -0.5;
			else if(dr < 0)
				return 0.5;
			else if(dl+dr == 0)
				return 0;
			
			if(type == LINEAR) {
				if(dl <= dr)
					return -0.5F + dl / (2.0F*dr);
					else return 0.5F - dr / (2.0F*dl);
			} else {
				if(dl <= dr)
					return -0.5F + dl / float(dl + dr);
				else return 0.5F - dr / float(dr + dl);
			}
		}
		
		template <typename T>
		static float minOffset(T c1, T c2, T c3, InterpolationType type) {
			T dl = c1 - c2;
			T dr = c3 - c2;
			
			// Handle special cases
			if(dl < 0)
				return -0.5;
			else if(dr < 0)
				return 0.5;
			else if(dl+dr == 0)
				return 0;
			
			if(type == LINEAR) {
				if(dl <= dr)
					return -0.5F + dl / (2.0F*dr);
				else return 0.5F - dr / (2.0F*dl);
			} else {
				if(dl <= dr)
					return -0.5F + dl / float(dl + dr);
				else return 0.5F - dr / float(dr + dl);
			}
		}
	};
	
	
}

#endif
