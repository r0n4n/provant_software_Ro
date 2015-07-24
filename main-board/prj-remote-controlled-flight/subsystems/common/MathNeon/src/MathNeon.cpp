/*
 * MathNeon.cpp
 *
 *  Created on: 24/7/2015
 *      Author: richard
 */

#include "MathNeon.h"
#include <math.h>
#ifdef __MATH_NEON
#include "arm_neon.h"
#endif

static const float __sinf_rng[2] = {
			2.0 / M_PI,
			M_PI / 2.0
		} ALIGN(16);

		static const float __sinf_lut[4] = {
			-0.00018365f,	//p7
			-0.16664831f,	//p3
			+0.00830636f,	//p5
			+0.99999661f,	//p1
		} ALIGN(16);
		const float __sincosf_rng[2] = {
			2.0 / M_PI,
			M_PI / 2.0
		};

		const float __sincosf_lut[8] = {
			-0.00018365f,	//p7
			-0.00018365f,	//p7
			+0.00830636f,	//p5
			+0.00830636f,	//p5
			-0.16664831f,	//p3
			-0.16664831f,	//p3
			+0.99999661f,	//p1
			+0.99999661f,	//p1
		};
		const float __sinfv_rng[2] = {
			2.0 / M_PI,
			M_PI / 2.0,
		};

		const float __sinfv_lut[4] = {
			-0.00018365f,	//p7
			-0.16664831f,	//p3
			+0.00830636f,	//p5
			+0.99999661f,	//p1
		};
		const float __tanf_rng[2] = {
			2.0 / M_PI,
			M_PI / 2.0
		};

		const float __tanf_lut[4] = {
			-0.00018365f,	//p7
			-0.16664831f,	//p3
			+0.00830636f,	//p5
			+0.99999661f,	//p1
		};
		const float __atanf_lut[4] = {
			-0.0443265554792128,	//p7
			-0.3258083974640975,	//p3
			+0.1555786518463281,	//p5
			+0.9997878412794807  	//p1
		};

		const float __atanf_pi_2 = M_PI_2;

		const float __atan2f_lut[4] = {
			-0.0443265554792128,	//p7
			-0.3258083974640975,	//p3
			+0.1555786518463281,	//p5
			+0.9997878412794807  	//p1
		};

		const float __atan2f_pi_2 = M_PI_2;

		const float __asinf_lut[4] = {
			0.105312459675071, 	//p7
			0.169303418571894,	//p3
			0.051599985887214, 	//p5
			0.999954835104825	//p1
		};

		const float __asinf_pi_2 = M_PI_2;

		const float __acosf_pi_2 = M_PI_2;

		const float __sinhf_rng[2] = {
			1.442695041f,
			0.693147180f
		};

		const float __sinhf_lut[16] = {
			0.00019578093328483123,	//p7
			0.00019578093328483123,	//p7
			0.0014122663401803872, 	//p6
			0.0014122663401803872, 	//p6
			0.008336936973260111, 	//p5
			0.008336936973260111, 	//p5
			0.04165989275009526, 	//p4
			0.04165989275009526, 	//p4
			0.16666570253074878, 	//p3
			0.16666570253074878, 	//p3
			0.5000006143673624, 	//p2
			0.5000006143673624, 	//p2
			1.000000059694879, 		//p1
			1.000000059694879, 		//p1
			0.9999999916728642,		//p0
			0.9999999916728642		//p0
		};

		const float __coshf_rng[2] = {
			1.442695041f,
			0.693147180f
		};

		const float __coshf_lut[16] = {
			0.00019578093328483123,	//p7
			0.00019578093328483123,	//p7
			0.0014122663401803872, 	//p6
			0.0014122663401803872, 	//p6
			0.008336936973260111, 	//p5
			0.008336936973260111, 	//p5
			0.04165989275009526, 	//p4
			0.04165989275009526, 	//p4
			0.16666570253074878, 	//p3
			0.16666570253074878, 	//p3
			0.5000006143673624, 	//p2
			0.5000006143673624, 	//p2
			1.000000059694879, 		//p1
			1.000000059694879, 		//p1
			0.9999999916728642,		//p0
			0.9999999916728642		//p0
		};
		const float __expf_rng[2] = {
			1.442695041f,
			0.693147180f
		};

		const float __expf_lut[8] = {
			0.9999999916728642,		//p0
			0.04165989275009526, 	//p4
			0.5000006143673624, 	//p2
			0.0014122663401803872, 	//p6
			1.000000059694879, 		//p1
			0.008336936973260111, 	//p5
			0.16666570253074878, 	//p3
			0.00019578093328483123	//p7
		};

		const float __logf_rng =  0.693147180f;

		const float __logf_lut[8] = {
			-2.295614848256274, 	//p0
			-2.470711633419806, 	//p4
			-5.686926051100417, 	//p2
			-0.165253547131978, 	//p6
			+5.175912446351073, 	//p1
			+0.844006986174912, 	//p5
			+4.584458825456749, 	//p3
			+0.014127821926000		//p7
		};

		const float __log10f_rng =  0.3010299957f;

		const float __log10f_lut[8] = {
			-0.99697286229624, 		//p0
			-1.07301643912502, 		//p4
			-2.46980061535534, 		//p2
			-0.07176870463131, 		//p6
			2.247870219989470, 		//p1
			0.366547581117400, 		//p5
			1.991005185100089, 		//p3
			0.006135635201050,		//p7
		};
		const float __powf_rng[2] = {
			1.442695041f,
			0.693147180f
		};

		const float __powf_lut[16] = {
			-2.295614848256274, 	//p0	log
			-2.470711633419806, 	//p4
			-5.686926051100417, 	//p2
			-0.165253547131978, 	//p6
			+5.175912446351073, 	//p1
			+0.844006986174912, 	//p5
			+4.584458825456749, 	//p3
			+0.014127821926000,		//p7
			0.9999999916728642,		//p0	exp
			0.04165989275009526, 	//p4
			0.5000006143673624, 	//p2
			0.0014122663401803872, 	//p6
			1.000000059694879, 		//p1
			0.008336936973260111, 	//p5
			0.16666570253074878, 	//p3
			0.00019578093328483123	//p7
		};

Math_Neon::Math_Neon() {
	// TODO Auto-generated constructor stub

}

Math_Neon::~Math_Neon() {
	// TODO Auto-generated destructor stub
}
void Math_Neon::enable_runfast()
{
#ifdef __arm__
	static const unsigned int x = 0x04086060;
	static const unsigned int y = 0x03000000;
	int r;
	asm volatile (
		"fmrx	%0, fpscr			\n\t"	//r0 = FPSCR
		"and	%0, %0, %1			\n\t"	//r0 = r0 & 0x04086060
		"orr	%0, %0, %2			\n\t"	//r0 = r0 | 0x03000000
		"fmxr	fpscr, %0			\n\t"	//FPSCR = r0
		: "=r"(r)
		: "r"(x), "r"(y)
	);
#endif
}

float Math_Neon::cosf_c(float x)
{
	return sinf_c(x + M_PI_2);
}

float Math_Neon::cosf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	float xx = x + M_PI_2;
	return sinf_neon_hfp(xx);
#endif
}

float Math_Neon::cosf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vdup.f32 d0, r0 		\n\t");
	cosf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return cosf_c(x);
#endif
};

float Math_Neon::sinf_c(float x)
{
	union {
		float 	f;
		int 	i;
	} ax;

	float r, a, b, xx;
	int m, n;

	ax.f = fabsf(x);

	//Range Reduction:
	m = (int) (ax.f * __sinf_rng[0]);
	ax.f = ax.f - (((float)m) * __sinf_rng[1]);

	//Test Quadrant
	n = m & 1;
	ax.f = ax.f - n * __sinf_rng[1];
	m = m >> 1;
	n = n ^ m;
	m = (x < 0.0);
	n = n ^ m;
	n = n << 31;
	ax.i = ax.i ^ n;

	//Taylor Polynomial (Estrins)
	xx = ax.f * ax.f;
	a = (__sinf_lut[0] * ax.f) * xx + (__sinf_lut[2] * ax.f);
	b = (__sinf_lut[1] * ax.f) * xx + (__sinf_lut[3] * ax.f);
	xx = xx * xx;
	r = b + a * xx;

	return r;
}

float Math_Neon::sinf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vld1.32 		d3, [%0]				\n\t"	//d3 = {invrange, range}
	"vdup.f32 		d0, d0[0]				\n\t"	//d0 = {x, x}
	"vabs.f32 		d1, d0					\n\t"	//d1 = {ax, ax}

	"vmul.f32 		d2, d1, d3[0]			\n\t"	//d2 = d1 * d3[0]
	"vcvt.u32.f32 	d2, d2					\n\t"	//d2 = (int) d2
	"vmov.i32	 	d5, #1					\n\t"	//d5 = 1
	"vcvt.f32.u32 	d4, d2					\n\t"	//d4 = (float) d2
	"vshr.u32 		d7, d2, #1				\n\t"	//d7 = d2 >> 1
	"vmls.f32 		d1, d4, d3[1]			\n\t"	//d1 = d1 - d4 * d3[1]

	"vand.i32 		d5, d2, d5				\n\t"	//d5 = d2 & d5
	"vclt.f32 		d18, d0, #0				\n\t"	//d18 = (d0 < 0.0)
	"vcvt.f32.u32 	d6, d5					\n\t"	//d6 = (float) d5
	"vmls.f32 		d1, d6, d3[1]			\n\t"	//d1 = d1 - d6 * d3[1]
	"veor.i32 		d5, d5, d7				\n\t"	//d5 = d5 ^ d7
	"vmul.f32 		d2, d1, d1				\n\t"	//d2 = d1*d1 = {x^2, x^2}

	"vld1.32 		{d16, d17}, [%1]		\n\t"	//q8 = {p7, p3, p5, p1}
	"veor.i32 		d5, d5, d18				\n\t"	//d5 = d5 ^ d18
	"vshl.i32 		d5, d5, #31				\n\t"	//d5 = d5 << 31
	"veor.i32 		d1, d1, d5				\n\t"	//d1 = d1 ^ d5

	"vmul.f32 		d3, d2, d2				\n\t"	//d3 = d2*d2 = {x^4, x^4}
	"vmul.f32 		q0, q8, d1[0]			\n\t"	//q0 = q8 * d1[0] = {p7x, p3x, p5x, p1x}
	"vmla.f32 		d1, d0, d2[0]			\n\t"	//d1 = d1 + d0*d2 = {p5x + p7x^3, p1x + p3x^3}
	"vmla.f32 		d1, d3, d1[0]			\n\t"	//d1 = d1 + d3*d0 = {...., p1x + p3x^3 + p5x^5 + p7x^7}

	"vmov.f32 		s0, s3					\n\t"	//s0 = s3
	:
	: "r"(__sinf_rng), "r"(__sinf_lut)
    : "q0", "q1", "q2", "q3", "q8", "q9"
	);
#endif
}

float Math_Neon::sinf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vdup.f32 d0, r0 		\n\t");
	sinf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return sinf_c(x);
#endif

};

void Math_Neon::sincosf_c( float x, float r[2])
{
	union {
		float 	f;
		int 	i;
	} ax, bx;

	float y;
	float a, b, c, d, xx, yy;
	int m, n, o, p;

	y = x + __sincosf_rng[1];
	ax.f = fabsf(x);
	bx.f = fabsf(y);

	//Range Reduction:
	m = (int) (ax.f * __sincosf_rng[0]);
	o = (int) (bx.f * __sincosf_rng[0]);
	ax.f = ax.f - (((float)m) * __sincosf_rng[1]);
	bx.f = bx.f - (((float)o) * __sincosf_rng[1]);

	//Test Quadrant
	n = m & 1;
	p = o & 1;
	ax.f = ax.f - n * __sincosf_rng[1];
	bx.f = bx.f - p * __sincosf_rng[1];
	m = m >> 1;
	o = o >> 1;
	n = n ^ m;
	p = p ^ o;
	m = (x < 0.0);
	o = (y < 0.0);
	n = n ^ m;
	p = p ^ o;
	n = n << 31;
	p = p << 31;
	ax.i = ax.i ^ n;
	bx.i = bx.i ^ p;

	//Taylor Polynomial
	xx = ax.f * ax.f;
	yy = bx.f * bx.f;
	r[0] = __sincosf_lut[0];
	r[1] = __sincosf_lut[1];
	r[0] = r[0] * xx + __sincosf_lut[2];
	r[1] = r[1] * yy + __sincosf_lut[3];
	r[0] = r[0] * xx + __sincosf_lut[4];
	r[1] = r[1] * yy + __sincosf_lut[5];
	r[0] = r[0] * xx + __sincosf_lut[6];
	r[1] = r[1] * yy + __sincosf_lut[7];
	r[0] = r[0] * ax.f;
	r[1] = r[1] * bx.f;

}

void Math_Neon::sincosf_neon_hfp(float x, float r[2])
{
//HACK: Assumes for softfp that r1 = x, and for hardfp that s0 = x.
#ifdef __MATH_NEON
	asm volatile (
	//{x, y} = {x, x + pi/2}
	"vdup.f32 		d1, d0[0]				\n\t"	//d1 = {x, x}
	"vld1.32 		d3, [%1]				\n\t"	//d3 = {invrange, range}
	"vadd.f32 		d0, d1, d3				\n\t"	//d0 = d1 + d3
	"vmov.f32 		s0, s2					\n\t"	//d0[0] = d1[0]
	"vabs.f32 		d1, d0					\n\t"	//d1 = {abs(x), abs(y)}

	//Range Reduction:
	"vmul.f32 		d2, d1, d3[0]			\n\t"	//d2 = d1 * d3[0]
	"vcvt.u32.f32 	d2, d2					\n\t"	//d2 = (int) d2
	"vcvt.f32.u32 	d4, d2					\n\t"	//d4 = (float) d2
	"vmls.f32 		d1, d4, d3[1]			\n\t"	//d1 = d1 - d4 * d3[1]

	//Checking Quadrant:
	//ax = ax - (k&1) * M_PI_2
	"vmov.i32	 	d4, #1					\n\t"	//d4 = 1
	"vand.i32	 	d4, d4, d2				\n\t"	//d4 = d4 & d2
	"vcvt.f32.u32 	d5, d4					\n\t"	//d5 = (float) d4
	"vmls.f32 		d1, d5, d3[1]			\n\t"	//d1 = d1 - d5 * d3[1]

	//ax = ax ^ ((k & 1) ^ (k >> 1) ^ (x < 0) << 31)
	"vshr.u32 		d3, d2, #1				\n\t"	//d3 = d2 >> 1
	"veor.i32 		d4, d4, d3				\n\t"	//d4 = d4 ^ d3
	"vclt.f32 		d3, d0, #0				\n\t"	//d3 = (d0 < 0.0)
	"veor.i32 		d4, d4, d3				\n\t"	//d4 = d4 ^ d3
	"vshl.i32 		d4, d4, #31				\n\t"	//d4 = d4 << 31
	"veor.i32 		d0, d1, d4				\n\t"	//d0 = d1 ^ d4

	//polynomial:
	"vldm 			%2!, {d2, d3}	 		\n\t"	//d2 = {p7, p7}, d3 = {p5, p5}, r3 += 4;
	"vmul.f32 		d1, d0, d0				\n\t"	//d1 = d0 * d0 = {x^2, y^2}
	"vldm 			%2!, {d4}				\n\t"	//d4 = {p3, p3}, r3 += 2;
	"vmla.f32 		d3, d2, d1				\n\t"	//d3 = d3 + d2 * d1;
	"vldm	 		%2!, {d5}				\n\t"	//d5 = {p1, p1}, r3 += 2;
	"vmla.f32 		d4, d3, d1				\n\t"	//d4 = d4 + d3 * d1;
	"vmla.f32 		d5, d4, d1				\n\t"	//d5 = d5 + d4 * d1;
	"vmul.f32 		d5, d5, d0				\n\t"	//d5 = d5 * d0;

	"vstm.f32 		%0, {d5}				\n\t"	//r[0] = d5[0], r[1]=d5[1];

	: "+r"(r)
	: "r"(__sincosf_rng), "r"(__sincosf_lut)
    : "d0", "d1", "d2", "d3", "d4", "d5"
	);
#else
	sincosf_c(x, r);
#endif
}

void Math_Neon::sincosf_neon_sfp(float x, float r[2])
{
#ifdef __MATH_NEON
	asm volatile ("vdup.f32 d0, r0 		\n\t");
	sincosf_neon_hfp(x, r);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
    sincosf_c(x, r);
#endif
};

void Math_Neon::sinfv_c(float *x, int n, float *r)
{
	union {
		float 	f;
		int 	i;
	} ax, bx;

	float aa, ab, ba, bb, axx, bxx;
	int am, bm, an, bn;

	if (n & 0x1) {
		*r++ = sinf_c(*x++);
		n--;
	}

	float rng0 = __sinfv_rng[0];
	float rng1 = __sinfv_rng[1];

	while(n > 0){

		float x0 = *x++;
		float x1 = *x++;

		ax.f = fabsf(x0);
		bx.f = fabsf(x1);

		//Range Reduction:
		am = (int) (ax.f * rng0);
		bm = (int) (bx.f * rng0);

		ax.f = ax.f - (((float)am) * rng1);
		bx.f = bx.f - (((float)bm) * rng1);

		//Test Quadrant
		an = am & 1;
		bn = bm & 1;
		ax.f = ax.f - an * rng1;
		bx.f = bx.f - bn * rng1;
		am = (am & 2) >> 1;
		bm = (bm & 2) >> 1;
		ax.i = ax.i ^ ((an ^ am ^ (x0 < 0)) << 31);
		bx.i = bx.i ^ ((bn ^ bm ^ (x1 < 0)) << 31);

		//Taylor Polynomial (Estrins)
		axx = ax.f * ax.f;
		bxx = bx.f * bx.f;
		aa = (__sinfv_lut[0] * ax.f) * axx + (__sinfv_lut[2] * ax.f);
		ba = (__sinfv_lut[0] * bx.f) * bxx + (__sinfv_lut[2] * bx.f);
		ab = (__sinfv_lut[1] * ax.f) * axx + (__sinfv_lut[3] * ax.f);
		bb = (__sinfv_lut[1] * bx.f) * bxx + (__sinfv_lut[3] * bx.f);
		axx = axx * axx;
		bxx = bxx * bxx;
		*r++ = ab + aa * axx;
		*r++ = bb + ba * bxx;
		n -= 2;
	}


}

void Math_Neon::sinfv_neon(float *x, int n, float *r)
{
#ifdef __MATH_NEON
	asm volatile (""
	:
	:"r"(x), "r"(n)
	);
#else
	sinfv_c(x, n, r);
#endif
}

float Math_Neon::tanf_c(float x){

	union {
		float f;
		int i;
	} ax, c;

	float r, a, b, xx, cc, cx;
	int m;

	ax.f = fabsf(x);

	//Range Reduction:
	m = (int) (ax.f * __tanf_rng[0]);
	ax.f = ax.f - (((float)m) * __tanf_rng[1]);

	//Test Quadrant
	ax.f = ax.f - (m & 1) * __tanf_rng[1];
	ax.i = ax.i ^ ((*(int*)&x) & 0x80000000);

	//Taylor Polynomial (Estrins)
	xx = ax.f * ax.f;
	a = (__tanf_lut[0] * ax.f) * xx + (__tanf_lut[2] * ax.f);
	b = (__tanf_lut[1] * ax.f) * xx + (__tanf_lut[3] * ax.f);
	xx = xx * xx;
	r = b + a * xx;

	//cosine
	c.f = 1.0 - r * r;

	//fast invsqrt approximation (2x newton iterations)
    cc = c.f;
	c.i = 0x5F3759DF - (c.i >> 1);		//VRSQRTE
	cx = cc * c.f;
	a = (3.0f - cx * c.f) / 2;			//VRSQRTS
	c.f = c.f * a;
	cx = cc * c.f;
	a = (3.0f - cx * c.f) / 2;
    c.f = c.f * a;

	r = r * c.f;

	return r;
}

float Math_Neon::tanf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32 		d0, d0[0]				\n\t"	//d0 = {x, x}
	"vabs.f32 		d1, d0					\n\t"	//d1 = {ax, ax}

	//Range Reduction:
	"vld1.32 		d3, [%0]				\n\t"	//d3 = {invrange, range}
	"vmul.f32 		d2, d1, d3[0]			\n\t"	//d2 = d1 * d3[0]
	"vcvt.u32.f32 	d2, d2					\n\t"	//d2 = (int) d2
	"vcvt.f32.u32 	d4, d2					\n\t"	//d4 = (float) d2
	"vmls.f32 		d1, d4, d3[1]			\n\t"	//d1 = d1 - d4 * d3[1]

	//Checking Quadrant:
	//ax = ax - (k&1) * M_PI_2
	"vmov.i32 		d4, #1					\n\t"	//d4 = 1
	"vand.i32 		d2, d2, d4				\n\t"	//d2 = d2 & d4
	"vcvt.f32.u32 	d2, d2					\n\t"	//d2 = (float) d2
	"vmls.f32 		d1, d2, d3[1]			\n\t"	//d1 = d1 - d2 * d3[1]

	//ax = ax ^ ( x.i & 0x800000000)
	"vmov.i32 		d4, #0x80000000			\n\t"	//d4 = 0x80000000
	"vand.i32 		d0, d0, d4				\n\t"	//d0 = d0 & d4
	"veor.i32 		d1, d1, d0				\n\t"	//d1 = d1 ^ d0

	//polynomial:
	"vmul.f32 		d2, d1, d1				\n\t"	//d2 = d1*d1 = {x^2, x^2}
	"vld1.32 		{d4, d5}, [%1]			\n\t"	//d4 = {p7, p3}, d5 = {p5, p1}
	"vmul.f32 		d3, d2, d2				\n\t"	//d3 = d2*d2 = {x^4, x^4}
	"vmul.f32 		q0, q2, d1[0]			\n\t"	//q0 = q2 * d1[0] = {p7x, p3x, p5x, p1x}
	"vmla.f32 		d1, d0, d2[0]			\n\t"	//d1 = d1 + d0*d2 = {p5x + p7x^3, p1x + p3x^3}
	"vmla.f32 		d1, d3, d1[0]			\n\t"	//d1 = d1 + d3*d0 = {..., p1x + p3x^3 + p5x^5 + p7x^7}

	//cosine
	"vmov.f32 		s1, #1.0				\n\t"	//d0[1] = 1.0
	"vmls.f32 		d0, d1, d1				\n\t"	//d0 = {..., 1.0 - sx*sx}

	//invsqrt approx
	"vmov.f32 		d2, d0					\n\t"	//d2 = d0
	"vrsqrte.f32 	d0, d0					\n\t"	//d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32 		d3, d0, d2				\n\t"	//d3 = d0 * d2
	"vrsqrts.f32 	d4, d3, d0				\n\t"	//d4 = (3 - d0 * d3) / 2
	"vmul.f32 		d0, d0, d4				\n\t"	//d0 = d0 * d4
	"vmul.f32 		d3, d0, d2				\n\t"	//d3 = d0 * d2
	"vrsqrts.f32 	d4, d3, d0				\n\t"	//d4 = (3 - d0 * d3) / 2
	"vmul.f32 		d0, d0, d4				\n\t"	//d0 = d0 * d4

	"vmul.f32 		d0, d0, d1				\n\t"	//d0 = d0 * d1

	"vmov.f32 		s0, s1					\n\t"	//s0 = s1

	:: "r"(__tanf_rng), "r"(__tanf_lut)
    : "d0", "d1", "d2", "d3", "d4", "d5"
	);
#endif
}

float Math_Neon::tanf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vdup.f32 d0, r0 		\n\t");
	tanf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return tanf_c(x);
#endif
};

float Math_Neon::atanf_c(float x)
{

	float a, b, r, xx;
	int m;

	union {
		float f;
		int i;
	} xinv, ax;

	ax.f = fabs(x);

	//fast inverse approximation (2x newton)
	xinv.f = ax.f;
	m = 0x3F800000 - (xinv.i & 0x7F800000);
	xinv.i = xinv.i + m;
	xinv.f = 1.41176471f - 0.47058824f * xinv.f;
	xinv.i = xinv.i + m;
	b = 2.0 - xinv.f * ax.f;
	xinv.f = xinv.f * b;
	b = 2.0 - xinv.f * ax.f;
	xinv.f = xinv.f * b;

	//if |x| > 1.0 -> ax = -1/ax, r = pi/2
	xinv.f = xinv.f + ax.f;
	a = (ax.f > 1.0f);
	ax.f = ax.f - a * xinv.f;
	r = a * __atanf_pi_2;

	//polynomial evaluation
	xx = ax.f * ax.f;
	a = (__atanf_lut[0] * ax.f) * xx + (__atanf_lut[2] * ax.f);
	b = (__atanf_lut[1] * ax.f) * xx + (__atanf_lut[3] * ax.f);
	xx = xx * xx;
	b = b + a * xx;
	r = r + b;

	//if x < 0 -> r = -r
	a = 2 * r;
	b = (x < 0.0f);
	r = r - a * b;

	return r;
}

float Math_Neon::atanf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32	 	d0, d0[0]				\n\t"	//d0 = {x, x};

	"vdup.f32	 	d4, %1					\n\t"	//d4 = {pi/2, pi/2};
	"vmov.f32	 	d6, d0					\n\t"	//d6 = d0;
	"vabs.f32	 	d0, d0					\n\t"	//d0 = fabs(d0) ;

	//fast reciporical approximation
	"vrecpe.f32		d1, d0					\n\t"	//d1 = ~ 1 / d0;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;


	//if |x| > 1.0 -> ax = -1/ax, r = pi/2
	"vadd.f32		d1, d1, d0				\n\t"	//d1 = d1 + d0;
	"vmov.f32	 	d2, #1.0				\n\t"	//d2 = 1.0;
	"vcgt.f32	 	d3, d0, d2				\n\t"	//d3 = (d0 > d2);
	"vshr.u32	 	d3, #31					\n\t"	//d3 = (d0 > d2);
	"vcvt.f32.u32	d3, d3					\n\t"	//d5 = (float) d3;
	"vmls.f32		d0, d1, d3[0]			\n\t"	//d0 = d0 - d1 * d3[0];
	"vmul.f32		d7, d4, d3[0] 			\n\t"	//d7 = d5 * d4;

	//polynomial:
	"vmul.f32 		d2, d0, d0				\n\t"	//d2 = d0*d0 = {ax^2, ax^2}
	"vld1.32 		{d4, d5}, [%0]			\n\t"	//d4 = {p7, p3}, d5 = {p5, p1}
	"vmul.f32 		d3, d2, d2				\n\t"	//d3 = d2*d2 = {x^4, x^4}
	"vmul.f32 		q0, q2, d0[0]			\n\t"	//q0 = q2 * d0[0] = {p7x, p3x, p5x, p1x}
	"vmla.f32 		d1, d0, d2[0]			\n\t"	//d1 = d1 + d0*d2[0] = {p5x + p7x^3, p1x + p3x^3}
	"vmla.f32 		d1, d3, d1[0]			\n\t"	//d1 = d1 + d3*d1[0] = {..., p1x + p3x^3 + p5x^5 + p7x^7}
	"vadd.f32 		d1, d1, d7				\n\t"	//d1 = d1 + d7

	"vadd.f32 		d2, d1, d1				\n\t"	//d2 = d1 + d1
	"vclt.f32	 	d3, d6, #0				\n\t"	//d3 = (d6 < 0)
	"vshr.u32	 	d3, #31					\n\t"	//d3 = (d0 > d2);
	"vcvt.f32.u32	d3, d3					\n\t"	//d3 = (float) d3
	"vmls.f32 		d1, d3, d2				\n\t"	//d1 = d1 - d2 * d3;

	"vmov.f32 		s0, s3					\n\t"	//s0 = s3

	:: "r"(__atanf_lut),  "r"(__atanf_pi_2)
    : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"
	);

#endif
}

float Math_Neon::atanf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vdup.f32 d0, r0 		\n\t");
	atanf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return atanf_c(x);
#endif
};

float Math_Neon::atan2f_c(float y, float x)
{
	float a, b, c, r, xx;
	int m;
	union {
		float f;
		int i;
	} xinv;

	//fast inverse approximation (2x newton)
	xx = fabs(x);
	xinv.f = xx;
	m = 0x3F800000 - (xinv.i & 0x7F800000);
	xinv.i = xinv.i + m;
	xinv.f = 1.41176471f - 0.47058824f * xinv.f;
	xinv.i = xinv.i + m;
	b = 2.0 - xinv.f * xx;
	xinv.f = xinv.f * b;
	b = 2.0 - xinv.f * xx;
	xinv.f = xinv.f * b;

	c = fabs(y * xinv.f);

	//fast inverse approximation (2x newton)
	xinv.f = c;
	m = 0x3F800000 - (xinv.i & 0x7F800000);
	xinv.i = xinv.i + m;
	xinv.f = 1.41176471f - 0.47058824f * xinv.f;
	xinv.i = xinv.i + m;
	b = 2.0 - xinv.f * c;
	xinv.f = xinv.f * b;
	b = 2.0 - xinv.f * c;
	xinv.f = xinv.f * b;

	//if |x| > 1.0 -> ax = -1/ax, r = pi/2
	xinv.f = xinv.f + c;
	a = (c > 1.0f);
	c = c - a * xinv.f;
	r = a * __atan2f_pi_2;

	//polynomial evaluation
	xx = c * c;
	a = (__atan2f_lut[0] * c) * xx + (__atan2f_lut[2] * c);
	b = (__atan2f_lut[1] * c) * xx + (__atan2f_lut[3] * c);
	xx = xx * xx;
	r = r + a * xx;
	r = r + b;

	//determine quadrant and test for small x.
	b = M_PI;
	b = b - 2.0f * r;
	r = r + (x < 0.0f) * b;
	b = (fabs(x) < 0.000001f);
	c = !b;
	r = c * r;
	r = r + __atan2f_pi_2 * b;
	b = r + r;
	r = r - (y < 0.0f) * b;

	return r;
}

float Math_Neon::atan2f_neon_hfp(float y, float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32	 	d17, d0[1]				\n\t"	//d17 = {x, x};
	"vdup.f32	 	d16, d0[0]				\n\t"	//d16 = {y, y};

	//1.0 / x
	"vrecpe.f32		d18, d17				\n\t"	//d16 = ~ 1 / d1;
	"vrecps.f32		d19, d18, d17			\n\t"	//d17 = 2.0 - d16 * d1;
	"vmul.f32		d18, d18, d19			\n\t"	//d16 = d16 * d17;
	"vrecps.f32		d19, d18, d17			\n\t"	//d17 = 2.0 - d16 * d1;
	"vmul.f32		d18, d18, d19			\n\t"	//d16 = d16 * d17;

	//y * (1.0 /x)
	"vmul.f32		d0, d16, d18			\n\t"	//d0 = d16 * d18;


	"vdup.f32	 	d4, %1					\n\t"	//d4 = {pi/2, pi/2};
	"vmov.f32	 	d6, d0					\n\t"	//d6 = d0;
	"vabs.f32	 	d0, d0					\n\t"	//d0 = fabs(d0) ;

	//fast reciporical approximation
	"vrecpe.f32		d1, d0					\n\t"	//d1 = ~ 1 / d0;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;

	//if |x| > 1.0 -> ax = 1/ax, r = pi/2
	"vadd.f32		d1, d1, d0				\n\t"	//d1 = d1 + d0;
	"vmov.f32	 	d2, #1.0				\n\t"	//d2 = 1.0;
	"vcgt.f32	 	d3, d0, d2				\n\t"	//d3 = (d0 > d2);
	"vcvt.f32.u32	d3, d3					\n\t"	//d3 = (float) d3;
	"vmls.f32		d0, d1, d3				\n\t"	//d0 = d0 - d1 * d3;
	"vmul.f32		d7, d3, d4				\n\t"	//d7 = d3 * d4;

	//polynomial:
	"vmul.f32 		d2, d0, d0				\n\t"	//d2 = d0*d0 = {ax^2, ax^2}
	"vld1.32 		{d4, d5}, [%0]			\n\t"	//d4 = {p7, p3}, d5 = {p5, p1}
	"vmul.f32 		d3, d2, d2				\n\t"	//d3 = d2*d2 = {x^4, x^4}
	"vmul.f32 		q0, q2, d0[0]			\n\t"	//q0 = q2 * d0[0] = {p7x, p3x, p5x, p1x}
	"vmla.f32 		d1, d0, d2[0]			\n\t"	//d1 = d1 + d0*d2[0] = {p5x + p7x^3, p1x + p3x^3}
	"vmla.f32 		d1, d3, d1[0]			\n\t"	//d1 = d1 + d3*d1[0] = {..., p1x + p3x^3 + p5x^5 + p7x^7}
	"vadd.f32 		d1, d1, d7				\n\t"	//d1 = d1 + d7

	"vadd.f32 		d2, d1, d1				\n\t"	//d2 = d1 + d1
	"vclt.f32	 	d3, d6, #0				\n\t"	//d3 = (d6 < 0)
	"vcvt.f32.u32	d3, d3					\n\t"	//d3 = (float) d3
	"vmls.f32 		d1, d3, d2				\n\t"	//d1 = d1 - d2 * d3;

	"vmov.f32 		s0, s3					\n\t"	//s0 = s3

	:: "r"(__atan2f_lut),  "r"(__atan2f_pi_2)
    : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"
	);
#endif
}

float Math_Neon::atan2f_neon_sfp(float x, float y)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	asm volatile ("vmov.f32 s1, r1 		\n\t");
	atan2f_neon_hfp(x, y);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return atan2f_c(y, x);
#endif
};

float Math_Neon::asinf_c(float x)
{

	float a, b, c, d, r, ax;
	int m;

	union {
		float f;
		int i;
	} xx;

	ax = fabs(x);
	d = 0.5;
	d = d - ax*0.5;

	//fast invsqrt approx
	xx.f = d;
	xx.i = 0x5F3759DF - (xx.i >> 1);		//VRSQRTE
	c = d * xx.f;
	b = (3.0f - c * xx.f) * 0.5;		//VRSQRTS
	xx.f = xx.f * b;
	c = d * xx.f;
	b = (3.0f - c * xx.f) * 0.5;
    xx.f = xx.f * b;

	//fast inverse approx
	d = xx.f;
	m = 0x3F800000 - (xx.i & 0x7F800000);
	xx.i = xx.i + m;
	xx.f = 1.41176471f - 0.47058824f * xx.f;
	xx.i = xx.i + m;
	b = 2.0 - xx.f * d;
	xx.f = xx.f * b;
	b = 2.0 - xx.f * d;
	xx.f = xx.f * b;

	//if |x|>0.5 -> x = sqrt((1-x)/2)
	xx.f = xx.f - ax;
	a = (ax > 0.5f);
	d = __asinf_pi_2 * a;
	c = 1.0f - 3.0f * a;
	ax = ax + xx.f * a;

	//polynomial evaluation
	xx.f = ax * ax;
	a = (__asinf_lut[0] * ax) * xx.f + (__asinf_lut[2] * ax);
	b = (__asinf_lut[1] * ax) * xx.f + (__asinf_lut[3] * ax);
	xx.f = xx.f * xx.f;
	r = b + a * xx.f;
	r = d + c * r;

	a = r + r;
	b = (x < 0.0f);
	r = r - a * b;
	return r;
}

float Math_Neon::asinf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32	 	d0, d0[0]				\n\t"	//d0 = {x, x};
	"vdup.f32	 	d4, %1					\n\t"	//d4 = {pi/2, pi/2};
	"vmov.f32	 	d6, d0					\n\t"	//d6 = d0;
	"vabs.f32	 	d0, d0					\n\t"	//d0 = fabs(d0) ;

	"vmov.f32	 	d5, #0.5				\n\t"	//d5 = 0.5;
	"vmls.f32	 	d5, d0, d5				\n\t"	//d5 = d5 - d0*d5;

	//fast invsqrt approx
	"vmov.f32 		d1, d5					\n\t"	//d1 = d5
	"vrsqrte.f32 	d5, d5					\n\t"	//d5 = ~ 1.0 / sqrt(d5)
	"vmul.f32 		d2, d5, d1				\n\t"	//d2 = d5 * d1
	"vrsqrts.f32 	d3, d2, d5				\n\t"	//d3 = (3 - d5 * d2) / 2
	"vmul.f32 		d5, d5, d3				\n\t"	//d5 = d5 * d3
	"vmul.f32 		d2, d5, d1				\n\t"	//d2 = d5 * d1
	"vrsqrts.f32 	d3, d2, d5				\n\t"	//d3 = (3 - d5 * d3) / 2
	"vmul.f32 		d5, d5, d3				\n\t"	//d5 = d5 * d3

	//fast reciporical approximation
	"vrecpe.f32		d1, d5					\n\t"	//d1 = ~ 1 / d5;
	"vrecps.f32		d2, d1, d5				\n\t"	//d2 = 2.0 - d1 * d5;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;
	"vrecps.f32		d2, d1, d5				\n\t"	//d2 = 2.0 - d1 * d5;
	"vmul.f32		d5, d1, d2				\n\t"	//d5 = d1 * d2;

	//if |x| > 0.5 -> ax = sqrt((1-ax)/2), r = pi/2
	"vsub.f32		d5, d0, d5				\n\t"	//d5 = d0 - d5;
	"vmov.f32	 	d2, #0.5				\n\t"	//d2 = 0.5;
	"vcgt.f32	 	d3, d0, d2				\n\t"	//d3 = (d0 > d2);
	"vmov.f32		d1, #3.0 				\n\t"	//d5 = 3.0;
	"vshr.u32	 	d3, #31					\n\t"	//d3 = d3 >> 31;
	"vmov.f32		d16, #1.0 				\n\t"	//d16 = 1.0;
	"vcvt.f32.u32	d3, d3					\n\t"	//d3 = (float) d3;
	"vmls.f32		d0, d5, d3[0]			\n\t"	//d0 = d0 - d5 * d3[0];
	"vmul.f32		d7, d4, d3[0] 			\n\t"	//d7 = d5 * d4;
	"vmls.f32		d16, d1, d3[0] 			\n\t"	//d16 = d16 - d1 * d3;

	//polynomial:
	"vmul.f32 		d2, d0, d0				\n\t"	//d2 = d0*d0 = {ax^2, ax^2}
	"vld1.32 		{d4, d5}, [%0]			\n\t"	//d4 = {p7, p3}, d5 = {p5, p1}
	"vmul.f32 		d3, d2, d2				\n\t"	//d3 = d2*d2 = {x^4, x^4}
	"vmul.f32 		q0, q2, d0[0]			\n\t"	//q0 = q2 * d0[0] = {p7x, p3x, p5x, p1x}
	"vmla.f32 		d1, d0, d2[0]			\n\t"	//d1 = d1 + d0*d2[0] = {p5x + p7x^3, p1x + p3x^3}
	"vmla.f32 		d1, d3, d1[0]			\n\t"	//d1 = d1 + d3*d1[0] = {..., p1x + p3x^3 + p5x^5 + p7x^7}

	"vmla.f32 		d7, d1, d16				\n\t"	//d7 = d7 + d1*d16

	"vadd.f32 		d2, d7, d7				\n\t"	//d2 = d7 + d7
	"vclt.f32	 	d3, d6, #0				\n\t"	//d3 = (d6 < 0)
	"vshr.u32	 	d3, #31					\n\t"	//d3 = d3 >> 31;
	"vcvt.f32.u32	d3, d3					\n\t"	//d3 = (float) d3
	"vmls.f32 		d7, d2, d3[0]			\n\t"	//d7 = d7 - d2 * d3[0];

	"vmov.f32 		s0, s15					\n\t"	//s0 = s3

	:: "r"(__asinf_lut),  "r"(__asinf_pi_2)
    : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"
	);
#endif
}

float Math_Neon::asinf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	asinf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return asinf_c(x);
#endif
}

float Math_Neon::acosf_c(float x)
{
	return __acosf_pi_2 - asinf_c(x);
}

float Math_Neon::acosf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asinf_neon_hfp(x);
	asm volatile (
	"vdup.f32	 	d1, %0					\n\t"	//d1 = {pi/2, pi/2};
	"vsub.f32	 	d0, d1, d0				\n\t"	//d0 = d1 - d0;
	::"r"(__acosf_pi_2):
	);
#endif
}

float Math_Neon::acosf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	acosf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return acosf_c(x);
#endif
}

float Math_Neon::sinhf_c(float x)
{
	float a, b, xx;
	xx = -x;
	a = expf_c(x);
	b = expf_c(xx);
	a = a - b;
	a = a * 0.5f;
	return a;
}

float Math_Neon::sinhf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (
	"vdup.f32 		d0, d0[0]				\n\t"	//d0 = {x, x}
	"fnegs 			s1, s1					\n\t"	//s1 = -s1

	//Range Reduction:
	"vld1.32 		d2, [%0]				\n\t"	//d2 = {invrange, range}
	"vld1.32 		{d16, d17}, [%1]!		\n\t"
	"vmul.f32 		d6, d0, d2[0]			\n\t"	//d6 = d0 * d2[0]
	"vcvt.s32.f32 	d6, d6					\n\t"	//d6 = (int) d6
	"vld1.32 		{d18}, [%1]!			\n\t"
	"vcvt.f32.s32 	d1, d6					\n\t"	//d1 = (float) d6
	"vld1.32 		{d19}, [%1]!			\n\t"
	"vmls.f32 		d0, d1, d2[1]			\n\t"	//d0 = d0 - d1 * d2[1]
	"vld1.32 		{d20}, [%1]!			\n\t"

	//polynomial:
	"vmla.f32 		d17, d16, d0			\n\t"	//d17 = d17 + d16 * d0;
	"vld1.32 		{d21}, [%1]!			\n\t"
	"vmla.f32 		d18, d17, d0			\n\t"	//d18 = d18 + d17 * d0;
	"vld1.32 		{d22}, [%1]!			\n\t"
	"vmla.f32 		d19, d18, d0			\n\t"	//d19 = d19 + d18 * d0;
	"vld1.32 		{d23}, [%1]!			\n\t"
	"vmla.f32 		d20, d19, d0			\n\t"	//d20 = d20 + d19 * d0;
	"vmla.f32 		d21, d20, d0			\n\t"	//d21 = d21 + d20 * d0;
	"vmla.f32 		d22, d21, d0			\n\t"	//d22 = d22 + d21 * d0;
	"vmla.f32 		d23, d22, d0			\n\t"	//d23 = d23 + d22 * d0;

	//multiply by 2 ^ m
	"vshl.i32 		d6, d6, #23				\n\t"	//d6 = d6 << 23
	"vadd.i32 		d0, d23, d6				\n\t"	//d0 = d22 + d6

	"vdup.f32 		d2, d0[1]				\n\t"	//d2 = s1
	"vmov.f32 		d1, #0.5				\n\t"	//d1 = 0.5
	"vsub.f32 		d0, d0, d2				\n\t"	//d0 = d0 - d2
	"vmul.f32 		d0, d1					\n\t"	//d0 = d0 * d1

	:: "r"(__sinhf_rng), "r"(__sinhf_lut)
    : "d0", "d1", "q1", "q2", "d6"
	);

#endif
}

float Math_Neon::sinhf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	sinhf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return sinhf_c(x);
#endif
};

float Math_Neon::coshf_c(float x)
{
	float a, b, xx;
	xx = -x;
	a = expf_c(x);
	b = expf_c(xx);
	a = a * 0.5f;
	a = a + 0.5f * b;
	return a;
}

float Math_Neon::coshf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (
	"vdup.f32 		d0, d0[0]				\n\t"	//d0 = {x, x}
	"fnegs 			s1, s1					\n\t"	//s1 = -s1

	//Range Reduction:
	"vld1.32 		d2, [%0]				\n\t"	//d2 = {invrange, range}
	"vld1.32 		{d16, d17}, [%1]!		\n\t"
	"vmul.f32 		d6, d0, d2[0]			\n\t"	//d6 = d0 * d2[0]
	"vcvt.s32.f32 	d6, d6					\n\t"	//d6 = (int) d6
	"vld1.32 		{d18}, [%1]!			\n\t"
	"vcvt.f32.s32 	d1, d6					\n\t"	//d1 = (float) d6
	"vld1.32 		{d19}, [%1]!			\n\t"
	"vmls.f32 		d0, d1, d2[1]			\n\t"	//d0 = d0 - d1 * d2[1]
	"vld1.32 		{d20}, [%1]!			\n\t"

	//polynomial:
	"vmla.f32 		d17, d16, d0			\n\t"	//d17 = d17 + d16 * d0;
	"vld1.32 		{d21}, [%1]!			\n\t"
	"vmla.f32 		d18, d17, d0			\n\t"	//d18 = d18 + d17 * d0;
	"vld1.32 		{d22}, [%1]!			\n\t"
	"vmla.f32 		d19, d18, d0			\n\t"	//d19 = d19 + d18 * d0;
	"vld1.32 		{d23}, [%1]!			\n\t"
	"vmla.f32 		d20, d19, d0			\n\t"	//d20 = d20 + d19 * d0;
	"vmla.f32 		d21, d20, d0			\n\t"	//d21 = d21 + d20 * d0;
	"vmla.f32 		d22, d21, d0			\n\t"	//d22 = d22 + d21 * d0;
	"vmla.f32 		d23, d22, d0			\n\t"	//d23 = d23 + d22 * d0;

	//multiply by 2 ^ m
	"vshl.i32 		d6, d6, #23				\n\t"	//d6 = d6 << 23
	"vadd.i32 		d0, d23, d6				\n\t"	//d0 = d22 + d6

	"vdup.f32 		d2, d0[1]				\n\t"	//d2 = s1
	"vmov.f32 		d1, #0.5				\n\t"	//d1 = 0.5
	"vadd.f32 		d0, d0, d2				\n\t"	//d0 = d0 + d2
	"vmul.f32 		d0, d1					\n\t"	//d0 = d0 * d1

	:: "r"(__coshf_rng), "r"(__coshf_lut)
    : "d0", "d1", "q1", "q2", "d6"
	);

#endif
}

float Math_Neon::coshf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	coshf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return coshf_c(x);
#endif
};

float Math_Neon::tanhf_c(float x)
{
	float a, b, c;
	int m;
	union{
		float 	f;
		int 	i;
	} xx;

	x = 2.0f * x;
	a = expf_c(x);
	c = a + 1.0f;

	//reciporical approx.
	xx.f = c;
	m = 0x3F800000 - (xx.i & 0x7F800000);
	xx.i = xx.i + m;
	xx.f = 1.41176471f - 0.47058824f * xx.f;
	xx.i = xx.i + m;
	b = 2.0 - xx.f * c;
	xx.f = xx.f * b;
	b = 2.0 - xx.f * c;
	xx.f = xx.f * b;
	c = a - 1.0;
	xx.f *= c;
	return xx.f;
}

float Math_Neon::tanhf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vadd.f32 d0, d0, d0 		\n\t");
	expf_neon_hfp(x);
	asm volatile (
	"vmov.f32 		d2, #1.0 				\n\t"
	"vsub.f32 		d3, d0, d2 				\n\t"
	"vadd.f32 		d0, d0, d2 				\n\t"

	"vrecpe.f32		d1, d0					\n\t"	//d1 = ~ 1 / d0;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d0, d1, d2				\n\t"	//d0 = d1 * d2;
	"vmul.f32		d0, d0, d3				\n\t"	//d0 = d0 * d3;
	::: "d0", "d1", "d2", "d3"
	);
#endif
}

float Math_Neon::tanhf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	tanhf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return tanhf_c(x);
#endif
};

float Math_Neon::expf_c(float x)
{
	float a, b, c, d, xx;
	int m;

	union {
		float   f;
		int 	i;
	} r;

	//Range Reduction:
	m = (int) (x * __expf_rng[0]);
	x = x - ((float) m) * __expf_rng[1];

	//Taylor Polynomial (Estrins)
	a = (__expf_lut[4] * x) + (__expf_lut[0]);
	b = (__expf_lut[6] * x) + (__expf_lut[2]);
	c = (__expf_lut[5] * x) + (__expf_lut[1]);
	d = (__expf_lut[7] * x) + (__expf_lut[3]);
	xx = x * x;
	a = a + b * xx;
	c = c + d * xx;
	xx = xx* xx;
	r.f = a + c * xx;

	//multiply by 2 ^ m
	m = m << 23;
	r.i = r.i + m;

	return r.f;
}

float Math_Neon::expf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (
	"vdup.f32 		d0, d0[0]				\n\t"	//d0 = {x, x}

	//Range Reduction:
	"vld1.32 		d2, [%0]				\n\t"	//d2 = {invrange, range}
	"vmul.f32 		d6, d0, d2[0]			\n\t"	//d6 = d0 * d2[0]
	"vcvt.s32.f32 	d6, d6					\n\t"	//d6 = (int) d6
	"vcvt.f32.s32 	d1, d6					\n\t"	//d1 = (float) d6
	"vmls.f32 		d0, d1, d2[1]			\n\t"	//d0 = d0 - d1 * d2[1]

	//polynomial:
	"vmul.f32 		d1, d0, d0				\n\t"	//d1 = d0*d0 = {x^2, x^2}
	"vld1.32 		{d2, d3, d4, d5}, [%1]	\n\t"	//q1 = {p0, p4, p2, p6}, q2 = {p1, p5, p3, p7} ;
	"vmla.f32 		q1, q2, d0[0]			\n\t"	//q1 = q1 + q2 * d0[0]
	"vmla.f32 		d2, d3, d1[0]			\n\t"	//d2 = d2 + d3 * d1[0]
	"vmul.f32 		d1, d1, d1				\n\t"	//d1 = d1 * d1 = {x^4, x^4}
	"vmla.f32 		d2, d1, d2[1]			\n\t"	//d2 = d2 + d1 * d2[1]

	//multiply by 2 ^ m
	"vshl.i32 		d6, d6, #23				\n\t"	//d6 = d6 << 23
	"vadd.i32 		d0, d2, d6				\n\t"	//d0 = d2 + d6

	:: "r"(__expf_rng), "r"(__expf_lut)
    : "d0", "d1", "q1", "q2", "d6"
	);
#endif
}

float Math_Neon::expf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	expf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return expf_c(x);
#endif
};

float Math_Neon::logf_c(float x)
{
	float a, b, c, d, xx;
	int m;

	union {
		float   f;
		int 	i;
	} r;

	//extract exponent
	r.f = x;
	m = (r.i >> 23);
	m = m - 127;
	r.i = r.i - (m << 23);

	//Taylor Polynomial (Estrins)
	xx = r.f * r.f;
	a = (__logf_lut[4] * r.f) + (__logf_lut[0]);
	b = (__logf_lut[6] * r.f) + (__logf_lut[2]);
	c = (__logf_lut[5] * r.f) + (__logf_lut[1]);
	d = (__logf_lut[7] * r.f) + (__logf_lut[3]);
	a = a + b * xx;
	c = c + d * xx;
	xx = xx * xx;
	r.f = a + c * xx;

	//add exponent
	r.f = r.f + ((float) m) * __logf_rng;

	return r.f;
}

float Math_Neon::logf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32		d0, d0[0]				\n\t"	//d0 = {x,x};

	//extract exponent
	"vmov.i32		d2, #127				\n\t"	//d2 = 127;
	"vshr.u32		d6, d0, #23				\n\t"	//d6 = d0 >> 23;
	"vsub.i32		d6, d6, d2				\n\t"	//d6 = d6 - d2;
	"vshl.u32		d1, d6, #23				\n\t"	//d1 = d6 << 23;
	"vsub.i32		d0, d0, d1				\n\t"	//d0 = d0 + d1;

	//polynomial:
	"vmul.f32 		d1, d0, d0				\n\t"	//d1 = d0*d0 = {x^2, x^2}
	"vld1.32 		{d2, d3, d4, d5}, [%1]	\n\t"	//q1 = {p0, p4, p2, p6}, q2 = {p1, p5, p3, p7} ;
	"vmla.f32 		q1, q2, d0[0]			\n\t"	//q1 = q1 + q2 * d0[0]
	"vmla.f32 		d2, d3, d1[0]			\n\t"	//d2 = d2 + d3 * d1[0]
	"vmul.f32 		d1, d1, d1				\n\t"	//d1 = d1 * d1 = {x^4, x^4}
	"vmla.f32 		d2, d1, d2[1]			\n\t"	//d2 = d2 + d1 * d2[1]

	//add exponent
	"vdup.32 		d7, %0					\n\t"	//d7 = {rng, rng}
	"vcvt.f32.s32 	d6, d6					\n\t"	//d6 = (float) d6
	"vmla.f32 		d2, d6, d7				\n\t"	//d2 = d2 + d6 * d7

	"vmov.f32 		s0, s4					\n\t"	//s0 = s4

	:: "r"(__logf_rng), "r"(__logf_lut)
    : "d0", "d1", "q1", "q2", "d6", "d7"
	);
#endif
}

float Math_Neon::logf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	logf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return logf_c(x);
#endif
};

float Math_Neon::log10f_c(float x)
{
	float a, b, c, d, xx;
	int m;

	union {
		float   f;
		int 	i;
	} r;

	//extract exponent
	r.f = x;
	m = (r.i >> 23);
	m = m - 127;
	r.i = r.i - (m << 23);

	//Taylor Polynomial (Estrins)
	xx = r.f * r.f;
	a = (__log10f_lut[4] * r.f) + (__log10f_lut[0]);
	b = (__log10f_lut[6] * r.f) + (__log10f_lut[2]);
	c = (__log10f_lut[5] * r.f) + (__log10f_lut[1]);
	d = (__log10f_lut[7] * r.f) + (__log10f_lut[3]);
	a = a + b * xx;
	c = c + d * xx;
	xx = xx * xx;
	r.f = a + c * xx;

	//add exponent
	r.f = r.f + ((float) m) * __log10f_rng;

	return r.f;
}

float Math_Neon::log10f_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32		d0, d0[0]				\n\t"	//d0 = {x,x};

	//extract exponent
	"vmov.i32		d2, #127				\n\t"	//d2 = 127;
	"vshr.u32		d6, d0, #23				\n\t"	//d6 = d0 >> 23;
	"vsub.i32		d6, d6, d2				\n\t"	//d6 = d6 - d2;
	"vshl.u32		d1, d6, #23				\n\t"	//d1 = d6 << 23;
	"vsub.i32		d0, d0, d1				\n\t"	//d0 = d0 + d1;

	//polynomial:
	"vmul.f32 		d1, d0, d0				\n\t"	//d1 = d0*d0 = {x^2, x^2}
	"vld1.32 		{d2, d3, d4, d5}, [%1]	\n\t"	//q1 = {p0, p4, p2, p6}, q2 = {p1, p5, p3, p7} ;
	"vmla.f32 		q1, q2, d0[0]			\n\t"	//q1 = q1 + q2 * d0[0]
	"vmla.f32 		d2, d3, d1[0]			\n\t"	//d2 = d2 + d3 * d1[0]
	"vmul.f32 		d1, d1, d1				\n\t"	//d1 = d1 * d1 = {x^4, x^4}
	"vmla.f32 		d2, d1, d2[1]			\n\t"	//d2 = d2 + d1 * d2[1]

	//add exponent
	"vdup.32 		d7, %0					\n\t"	//d7 = {rng, rng}
	"vcvt.f32.s32 	d6, d6					\n\t"	//d6 = (float) d6
	"vmla.f32 		d2, d6, d7				\n\t"	//d2 = d2 + d6 * d7

	"vmov.f32 		s0, s4					\n\t"	//s0 = s4

	:: "r"(__log10f_rng), "r"(__log10f_lut)
    : "d0", "d1", "q1", "q2", "d6", "d7"
	);
#endif
}

float Math_Neon::log10f_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	log10f_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return log10f_c(x);
#endif
};

float Math_Neon::powf_c(float x, float n)
{
	float a, b, c, d, xx;
	int m;

	union {
		float   f;
		int 	i;
	} r;

	//extract exponent
	r.f = x;
	m = (r.i >> 23);
	m = m - 127;
	r.i = r.i - (m << 23);

	//Taylor Polynomial (Estrins)
	xx = r.f * r.f;
	a = (__powf_lut[4] * r.f) + (__powf_lut[0]);
	b = (__powf_lut[6] * r.f) + (__powf_lut[2]);
	c = (__powf_lut[5] * r.f) + (__powf_lut[1]);
	d = (__powf_lut[7] * r.f) + (__powf_lut[3]);
	a = a + b * xx;
	c = c + d * xx;
	xx = xx * xx;
	r.f = a + c * xx;

	//add exponent
	r.f = r.f + ((float) m) * __powf_rng[1];

	r.f = r.f * n;


	//Range Reduction:
	m = (int) (r.f * __powf_rng[0]);
	r.f = r.f - ((float) m) * __powf_rng[1];

	//Taylor Polynomial (Estrins)
	a = (__powf_lut[12] * r.f) + (__powf_lut[8]);
	b = (__powf_lut[14] * r.f) + (__powf_lut[10]);
	c = (__powf_lut[13] * r.f) + (__powf_lut[9]);
	d = (__powf_lut[15] * r.f) + (__powf_lut[11]);
	xx = r.f * r.f;
	a = a + b * xx;
	c = c + d * xx;
	xx = xx* xx;
	r.f = a + c * xx;

	//multiply by 2 ^ m
	m = m << 23;
	r.i = r.i + m;

	return r.f;
}

float Math_Neon::powf_neon_hfp(float x, float n)
{
#ifdef __MATH_NEON
	asm volatile (

	"vdup.f32		d16, d0[1]				\n\t"	//d16 = {y,y};
	"vdup.f32		d0, d0[0]				\n\t"	//d0 = {x,x};

	//extract exponent
	"vmov.i32		d2, #127				\n\t"	//d2 = 127;
	"vshr.u32		d6, d0, #23				\n\t"	//d6 = d0 >> 23;
	"vsub.i32		d6, d6, d2				\n\t"	//d6 = d6 - d2;
	"vshl.u32		d1, d6, #23				\n\t"	//d1 = d6 << 23;
	"vsub.i32		d0, d0, d1				\n\t"	//d0 = d0 + d1;

	//polynomial:
	"vmul.f32 		d1, d0, d0				\n\t"	//d1 = d0*d0 = {x^2, x^2}
	"vld1.32 		{d2, d3, d4, d5}, [%1]!	\n\t"	//q1 = {p0, p4, p2, p6}, q2 = {p1, p5, p3, p7} ;
	"vmla.f32 		q1, q2, d0[0]			\n\t"	//q1 = q1 + q2 * d0[0]
	"vmla.f32 		d2, d3, d1[0]			\n\t"	//d2 = d2 + d3 * d1[0]
	"vmul.f32 		d1, d1, d1				\n\t"	//d1 = d1 * d1 = {x^4, x^4}
	"vmla.f32 		d2, d1, d2[1]			\n\t"	//d2 = d2 + d1 * d2[1]

	//add exponent
	"vld1.32 		d7, [%0]				\n\t"	//d7 = {invrange, range}
	"vcvt.f32.s32 	d6, d6					\n\t"	//d6 = (float) d6
	"vmla.f32 		d2, d6, d7[1]			\n\t"	//d2 = d2 + d6 * d7[1]

	"vdup.f32 		d0, d2[0]				\n\t"	//d0 = d2[0]
	"vmul.f32 		d0, d0, d16				\n\t"	//d0 = d0 * d16

	//Range Reduction:
	"vmul.f32 		d6, d0, d7[0]			\n\t"	//d6 = d0 * d7[0]
	"vcvt.u32.f32 	d6, d6					\n\t"	//d6 = (int) d6
	"vcvt.f32.u32 	d1, d6					\n\t"	//d1 = (float) d6
	"vmls.f32 		d0, d1, d7[1]			\n\t"	//d0 = d0 - d1 * d7[1]

	//polynomial:
	"vmul.f32 		d1, d0, d0				\n\t"	//d1 = d0*d0 = {x^2, x^2}
	"vld1.32 		{d2, d3, d4, d5}, [%1]	\n\t"	//q1 = {p0, p4, p2, p6}, q2 = {p1, p5, p3, p7} ;
	"vmla.f32 		q1, q2, d0[0]			\n\t"	//q1 = q1 + q2 * d0[0]
	"vmla.f32 		d2, d3, d1[0]			\n\t"	//d2 = d2 + d3 * d1[0]
	"vmul.f32 		d1, d1, d1				\n\t"	//d1 = d1 * d1 = {x^4, x^4}
	"vmla.f32 		d2, d1, d2[1]			\n\t"	//d2 = d2 + d1 * d2[1]

	//multiply by 2 ^ m
	"vshl.i32 		d6, d6, #23				\n\t"	//d6 = d6 << 23
	"vadd.i32 		d0, d2, d6				\n\t"	//d0 = d2 + d6


	:: "r"(__powf_rng), "r"(__powf_lut)
    : "d0", "d1", "d2","d3", "d4", "d5", "d6", "d7"
	);
#endif
}

float Math_Neon::powf_neon_sfp(float x, float n)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	asm volatile ("vmov.f32 s1, r1 		\n\t");
	powf_neon_hfp(x, n);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return powf_c(x, n);
#endif
};

float Math_Neon::floorf_c(float x)
{
	int n;
	float r;
	n = (int) x;
	r = (float) n;
	r = r - (r > x);
	return r;
}

float Math_Neon::floorf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (
	"vcvt.s32.f32 	d1, d0					\n\t"	//d1 = (int) d0;
	"vcvt.f32.s32 	d1, d1					\n\t"	//d1 = (float) d1;
	"vcgt.f32 		d0, d1, d0				\n\t"	//d0 = (d1 > d0);
	"vshr.u32 		d0, #31					\n\t"	//d0 = d0 >> 31;
	"vcvt.f32.u32 	d0, d0					\n\t"	//d0 = (float) d0;
	"vsub.f32 		d0, d1, d0				\n\t"	//d0 = d1 - d0;
	::: "d0", "d1"
	);
#endif
}

float Math_Neon::floorf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	floorf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return floorf_c(x);
#endif
};

float Math_Neon::ceilf_c(float x)
{
	int n;
	float r;
	n = (int) x;
	r = (float) n;
	r = r + (x > r);
	return r;
}

float Math_Neon::ceilf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vcvt.s32.f32 	d1, d0					\n\t"	//d1 = (int) d0;
	"vcvt.f32.s32 	d1, d1					\n\t"	//d1 = (float) d1;
	"vcgt.f32 		d0, d0, d1				\n\t"	//d0 = (d0 > d1);
	"vshr.u32 		d0, #31					\n\t"	//d0 = d0 >> 31;
	"vcvt.f32.u32 	d0, d0					\n\t"	//d0 = (float) d0;
	"vadd.f32 		d0, d1, d0				\n\t"	//d0 = d1 + d0;

	::: "d0", "d1"
	);

#endif
}

float Math_Neon::ceilf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	ceilf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return ceilf_c(x);
#endif
};

float Math_Neon::fabsf_c(float x)
{
	union {
		int i;
		float f;
	} xx;

	xx.f = x;
	xx.i = xx.i & 0x7FFFFFFF;
	return xx.f;
}

float Math_Neon::fabsf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (
	"fabss	 		s0, s0					\n\t"	//s0 = fabs(s0)
	);
#endif
}

float Math_Neon::fabsf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (
	"bic	 		r0, r0, #0x80000000		\n\t"	//r0 = r0 & ~(1 << 31)
	);
#else
	return fabsf_c(x);
#endif
}

float Math_Neon::ldexpf_c(float m, int e)
{
	union {
		float 	f;
		int 	i;
	} r;
	r.f = m;
	r.i += (e << 23);
	return r.f;
}

float Math_Neon::ldexpf_neon_hfp(float m, int e)
{
#ifdef __MATH_NEON
	float r;
	asm volatile (
	"lsl 			r0, r0, #23				\n\t"	//r0 = r0 << 23
	"vdup.i32 		d1, r0					\n\t"	//d1 = {r0, r0}
	"vadd.i32 		d0, d0, d1				\n\t"	//d0 = d0 + d1
	::: "d0", "d1"
	);
#endif
}

float Math_Neon::ldexpf_neon_sfp(float m, int e)
{
#ifdef __MATH_NEON
	float r;
	asm volatile (
	"lsl 			r1, r1, #23				\n\t"	//r1 = r1 << 23
	"vdup.f32 		d0, r0					\n\t"	//d0 = {r0, r0}
	"vdup.i32 		d1, r1					\n\t"	//d1 = {r1, r1}
	"vadd.i32 		d0, d0, d1				\n\t"	//d0 = d0 + d1
	"vmov.f32 		r0, s0					\n\t"	//r0 = s0
	::: "d0", "d1"
	);
#else
	return ldexpf_c(m,e);
#endif
}

float Math_Neon::fmodf_c(float x, float y)
{
	int n;
	union {
		float f;
		int   i;
	} yinv;
	float a;

	//fast reciporical approximation (4x Newton)
	yinv.f = y;
	n = 0x3F800000 - (yinv.i & 0x7F800000);
	yinv.i = yinv.i + n;
	yinv.f = 1.41176471f - 0.47058824f * yinv.f;
	yinv.i = yinv.i + n;
	a = 2.0 - yinv.f * y;
	yinv.f = yinv.f * a;
	a = 2.0 - yinv.f * y;
	yinv.f = yinv.f * a;
	a = 2.0 - yinv.f * y;
	yinv.f = yinv.f * a;
	a = 2.0 - yinv.f * y;
	yinv.f = yinv.f * a;

	n = (int)(x * yinv.f);
	x = x - ((float)n) * y;
	return x;
}

float Math_Neon::fmodf_neon_hfp(float x, float y)
{
#ifdef __MATH_NEON
	asm volatile (
	"vdup.f32 		d1, d0[1]					\n\t"	//d1[0] = y
	"vdup.f32 		d0, d0[0]					\n\t"	//d1[0] = y

	//fast reciporical approximation
	"vrecpe.f32 	d2, d1					\n\t"	//d2 = ~1.0 / d1
	"vrecps.f32		d3, d2, d1				\n\t"	//d3 = 2.0 - d2 * d1;
	"vmul.f32		d2, d2, d3				\n\t"	//d2 = d2 * d3;
	"vrecps.f32		d3, d2, d1				\n\t"	//d3 = 2.0 - d2 * d1;
	"vmul.f32		d2, d2, d3				\n\t"	//d2 = d2 * d3;
	"vrecps.f32		d3, d2, d1				\n\t"	//d3 = 2.0 - d2 * d1;
	"vmul.f32		d2, d2, d3				\n\t"	//d2 = d2 * d3;
	"vrecps.f32		d3, d2, d1				\n\t"	//d3 = 2.0 - d2 * d1;
	"vmul.f32		d2, d2, d3				\n\t"	//d2 = d2 * d3;

	"vmul.f32		d2, d2, d0				\n\t"	//d2 = d2 * d0;
	"vcvt.s32.f32	d2, d2					\n\t"	//d2 = (int) d2;
	"vcvt.f32.s32	d2, d2					\n\t"	//d2 = (float) d2;
	"vmls.f32		d0, d1, d2				\n\t"	//d0 = d0 - d1 * d2;

	::: "d0", "d1", "d2", "d3"
	);
#endif
}

float Math_Neon::fmodf_neon_sfp(float x, float y)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	asm volatile ("vmov.f32 s1, r1 		\n\t");
	fmodf_neon_hfp(x, y);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return fmodf_c(x,y);
#endif
};

float Math_Neon::modf_c(float x, int *i)
{
	int n;
	n = (int)x;
	*i = n;
	x = x - (float)n;
	return x;
}

float Math_Neon::modf_neon_hfp(float x, int *i)
{
#ifdef __MATH_NEON
	asm volatile (
	"vcvt.s32.f32	d1, d0					\n\t"	//d1 = (int) d0;
	"vcvt.f32.s32	d2, d1					\n\t"	//d2 = (float) d1;
	"vsub.f32		d0, d0, d2				\n\t"	//d0 = d0 - d2;
	"vstr.i32		s2, [r0]				\n\t"	//[r0] = d1[0]
	::: "d0", "d1", "d2"
	);
#endif
}

float Math_Neon::modf_neon_sfp(float x, int *i)
{
#ifdef __MATH_NEON
	asm volatile (
	"vdup.f32 		d0, r0					\n\t"	//d0 = {x, x}
	"vcvt.s32.f32	d1, d0					\n\t"	//d1 = (int) d0;
	"vcvt.f32.s32	d2, d1					\n\t"	//d2 = (float) d1;
	"vsub.f32		d0, d0, d2				\n\t"	//d0 = d0 - d2;
	"vstr.i32		s2, [r1]				\n\t"	//[r0] = d1[0]
	"vmov.f32 		r0, s0					\n\t"	//r0 = d0[0];
	::: "d0", "d1", "d2"
	);

#else
	return modf_c(x, i);
#endif
}

float Math_Neon::sqrtf_c(float x)
{

	float b, c;
	int m;
	union {
		float 	f;
		int 	i;
	} a;

	//fast invsqrt approx
	a.f = x;
	a.i = 0x5F3759DF - (a.i >> 1);		//VRSQRTE
	c = x * a.f;
	b = (3.0f - c * a.f) * 0.5;		//VRSQRTS
	a.f = a.f * b;
	c = x * a.f;
	b = (3.0f - c * a.f) * 0.5;
    a.f = a.f * b;

	//fast inverse approx
	x = a.f;
	m = 0x3F800000 - (a.i & 0x7F800000);
	a.i = a.i + m;
	a.f = 1.41176471f - 0.47058824f * a.f;
	a.i = a.i + m;
	b = 2.0 - a.f * x;
	a.f = a.f * b;
	b = 2.0 - a.f * x;
	a.f = a.f * b;

	return a.f;
}

float Math_Neon::sqrtf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	//fast invsqrt approx
	"vmov.f32 		d1, d0					\n\t"	//d1 = d0
	"vrsqrte.f32 	d0, d0					\n\t"	//d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32 		d2, d0, d1				\n\t"	//d2 = d0 * d1
	"vrsqrts.f32 	d3, d2, d0				\n\t"	//d3 = (3 - d0 * d2) / 2
	"vmul.f32 		d0, d0, d3				\n\t"	//d0 = d0 * d3
	"vmul.f32 		d2, d0, d1				\n\t"	//d2 = d0 * d1
	"vrsqrts.f32 	d3, d2, d0				\n\t"	//d4 = (3 - d0 * d3) / 2
	"vmul.f32 		d0, d0, d3				\n\t"	//d0 = d0 * d3

	//fast reciporical approximation
	"vrecpe.f32		d1, d0					\n\t"	//d1 = ~ 1 / d0;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d1, d1, d2				\n\t"	//d1 = d1 * d2;
	"vrecps.f32		d2, d1, d0				\n\t"	//d2 = 2.0 - d1 * d0;
	"vmul.f32		d0, d1, d2				\n\t"	//d0 = d1 * d2;

	::: "d0", "d1", "d2", "d3"
	);
#endif
}

float Math_Neon::sqrtf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	sqrtf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return sqrtf_c(x);
#endif
};

float Math_Neon::invsqrtf_c(float x)
{

	float b, c;
	union {
		float 	f;
		int 	i;
	} a;

	//fast invsqrt approx
	a.f = x;
	a.i = 0x5F3759DF - (a.i >> 1);		//VRSQRTE
	c = x * a.f;
	b = (3.0f - c * a.f) * 0.5;		//VRSQRTS
	a.f = a.f * b;
	c = x * a.f;
	b = (3.0f - c * a.f) * 0.5;
    a.f = a.f * b;

	return a.f;
}

float Math_Neon::invsqrtf_neon_hfp(float x)
{
#ifdef __MATH_NEON
	asm volatile (

	"vmov.f32 		d1, d0					\n\t"	//d1 = d0
	"vrsqrte.f32 	d0, d0					\n\t"	//d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32 		d2, d0, d1				\n\t"	//d2 = d0 * d1
	"vrsqrts.f32 	d3, d2, d0				\n\t"	//d3 = (3 - d0 * d2) / 2
	"vmul.f32 		d0, d0, d3				\n\t"	//d0 = d0 * d3
	"vmul.f32 		d2, d0, d1				\n\t"	//d2 = d0 * d1
	"vrsqrts.f32 	d3, d2, d0				\n\t"	//d4 = (3 - d0 * d3) / 2
	"vmul.f32 		d0, d0, d3				\n\t"	//d0 = d0 * d4

	::: "d0", "d1", "d2", "d3"
	);
#endif
}

float Math_Neon::invsqrtf_neon_sfp(float x)
{
#ifdef __MATH_NEON
	asm volatile ("vmov.f32 s0, r0 		\n\t");
	invsqrtf_neon_hfp(x);
	asm volatile ("vmov.f32 r0, s0 		\n\t");
#else
	return invsqrtf_c(x);
#endif
}
