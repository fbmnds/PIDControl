#load "FSharpChart.fsx"
open MSDN.FSharp.Charting

type PIDController<[<Measure>]'x,[<Measure>]'t> (pVal: float<1/'t>, 
                                                 iVal: float<1/'t>, 
                                                 dVal: float<'t>, 
                                                 xTarget: float<'x>, 
                                                 dt: float<'t>,
                                                 minCorr: float<'x> option, 
                                                 maxCorr: float<'x> option) = 
    let mutable pVal_ = pVal
    let mutable iVal_ = iVal
    let mutable dVal_ = dVal

    let mutable xTarget_ = xTarget
    let mutable dt_ = dt

    let mutable cumErr_  = 0.<_>
    let mutable lastErr_ = 0.<_>

    let mutable minCorr_ = minCorr
    let mutable maxCorr_ = maxCorr

    member x.pVal with get() = pVal_ and set v = pVal_ <- v
    member x.iVal with get() = iVal_ and set v = iVal_ <- v
    member x.dVal with get() = dVal_ and set v = dVal_ <- v
    
    member x.xTarget with get() = xTarget_ and set v = xTarget_ <- v
    member x.dt with get() = dt_ and set v = dt_ <- v
    member x.minCorr with get() = minCorr_ and set v = minCorr_ <- v
    member x.maxCorr with get() = maxCorr_ and set v = maxCorr_ <- v
    
    member x.Dampen corr = 
        let (a,b) = (x.minCorr, x.maxCorr)
        match a, b with  
        | Some a, Some b -> if corr < a then a
                            else if corr > b then b 
                            else corr
        | Some a, None -> if corr < a then a else corr
        | None, Some b -> if corr > b then b else corr
        | _ -> corr

    member x.Correction v =
        let mutable corr = 0.<_>
        /// P correction 
        let error = xTarget_ - v
        corr <- pVal * error * dt_
        /// I Correction
        cumErr_ <- cumErr_ + error
        corr <- corr + iVal_ * cumErr_ * dt_
        /// D Correction
        let slope = (error - lastErr_)/dt_
        corr <- corr + dVal * slope
        lastErr_ <- error
        /// dampening the correction value
        x.Dampen corr

    member x.Waypoints pos_ time_ eps maxTrials_ =
        seq { let pos = ref pos_
              let maxTrials = ref maxTrials_
              let posCorr = ref 0.<_>
              let time = ref time_
              yield (!time, !pos)
              while abs (!pos - xTarget_) > eps && !maxTrials > 0  do
                  posCorr := x.Correction !pos
                  pos := !pos + !posCorr
                  maxTrials := !maxTrials - 1
                  time := !time + dt_
                  yield (!time, !pos) }

[<Measure>]
type m

[<Measure>]
type s

let x = 0.<m>
let xTarget = 3.<m>
let startTime = 0.0<s>
let eps = 0.01<m>
let maxTrials = 1000

let pid = PIDController(1.<1/s>,0.01<1/s>,0.01<s>,xTarget,0.1<s>,None,None)

let w = (pid.Waypoints x startTime eps maxTrials)
        |> Seq.map (fun (x,y) -> ((float x), (float y)))
        |> List.ofSeq

for i in w do printfn "%A" i

type Chart = FSharpChart
Chart.Line w

