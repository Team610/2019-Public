package frc.util;

public class Colour {

    public static Colour PINK = new Colour(210/255.0, 75/255.0, 210/255.0);
    public static Colour RED = new Colour(1, 0, 0);
    public static Colour GREEN = new Colour(0, 1, 0);
    public static Colour BLACK = new Colour(0, 0, 0);

    private double r, g, b;

    public Colour(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    } 

    public void setRGB(Colour c) {
        double[] rgb = c.getRGB();
        this.r = rgb[0];
        this.g = rgb[1];
        this.b = rgb[2];
    }

    public void setRGB(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public double[] getRGB() {
        return new double[]{Util.clamp(r, 0, 1),Util.clamp(g, 0, 1),Util.clamp(b, 0, 1)};
    }

    public static Colour fromHSV(double hDegrees, double S, double V) {
		double R, G, B;
		double H = hDegrees;

		if (H < 0) {
			H += 360;
		}
		if (H >= 360) {
			H -= 360;
		}

		if (V <= 0) {
			R = G = B = 0;
		} else if (S <= 0) {
			R = G = B = V;
		} else {
			double hf = H / 60.0;
			int i = (int) Math.floor(hf);
			double f = hf - i;
			double pv = V * (1 - S);
			double qv = V * (1 - S * f);
			double tv = V * (1 - S * (1 - f));
			switch (i) {
				/* Red is dominant color */
				case 0 :
					R = V;
					G = tv;
					B = pv;
					break;
				/* Green is dominant color */
				case 1 :
					R = qv;
					G = V;
					B = pv;
					break;
				case 2 :
					R = pv;
					G = V;
					B = tv;
					break;
				/* Blue is the dominant color */
				case 3 :
					R = pv;
					G = qv;
					B = V;
					break;
				case 4 :
					R = tv;
					G = pv;
					B = V;
					break;
				/* Red is the dominant color */
				case 5 :
					R = V;
					G = pv;
					B = qv;
					break;
				/**
				 * Just in case we overshoot on our math by a little, we put
				 * these here. Since its a switch it won't slow us down at all
				 * to put these here
				 */
				case 6 :
					R = V;
					G = tv;
					B = pv;
					break;
				case -1 :
					R = V;
					G = pv;
					B = qv;
					break;
				/* The color is not defined, we should throw an error */
				default :
					/* Just pretend its black/white */
					R = G = B = V;
					break;
			}
		}

		return new Colour(R, G, B);
	}
}