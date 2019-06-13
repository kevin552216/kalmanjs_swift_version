
import Foundation


class KalmanFilter {
    
    var R : Float
    var Q : Float
    var A : Float
    var B : Float
    var C: Float
    var x : Float
    var cov : Float
    init(){
        self.R = 1
        self.Q = 1
        self.A = 1
        self.B = 0
        self.C = 1
        self.cov = Float.nan
        self.x = Float.nan
    }
    init(R : Float,
         Q :Float,
         A :Float,
         B :Float,
         C :Float){
        self.R = R
        self.Q = Q
        self.A = A
        self.B = B
        self.C = C
        self.cov = Float.nan
        self.x = Float.nan
    }
    public func filter(z:Float)->Float{
        let u:Float = 0.0
        if self.x.isNaN {
            self.x = (1 / self.C) * z
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        }
        else {
            let predX = self.predict(u: u)
            let predCov = self.uncertainty()
            let K = predCov * self.C * (1 / ((self.C * predCov * self.C) + self.Q))
            self.x = predX + K * (z - (self.C * predX))
            self.cov = predCov - (K * self.C * predCov)
        }
        return self.x
    }
    public func filter(z:Float, u:Float)->Float{
        if self.x.isNaN {
            self.x = (1 / self.C) * z
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        }
        else {
            let predX = self.predict(u: u)
            let predCov = self.uncertainty()
            let K = predCov * self.C * (1 / ((self.C * predCov * self.C) + self.Q))
            self.x = predX + K * (z - (self.C * predX))
            self.cov = predCov - (K * self.C * predCov)
        }
        return self.x
    }
    
    public func predict() -> Float{
        let u:Float = 0.0
        return self.A * self.x + self.B * u
    }
    public func predict(u:Float) -> Float{
        return self.A * self.x + self.B * u
    }
    public func uncertainty() -> Float{
        return self.A * self.cov * self.A + self.R
    }
    public func lastMeasurement()->Float{
        return self.x
    }
    public func setMeasurementNoise(noise:Float){
        self.Q = noise
    }
    public func setProcessNoise(noise:Float){
        self.R = noise
    }

}
