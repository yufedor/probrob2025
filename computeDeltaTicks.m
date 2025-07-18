#compute delta ticks with overflow correction for uint32 encoders
#converts absolute encoder readings into deltas, handling overflow
#ticks:	the reading of the encoders
#max_tick_value: the maximum allowed value
function delta_ticks = computeDeltaTicks(ticks, max_tick_value)
  delta_ticks = zeros(size(ticks));
  for i = 2:length(ticks)
    raw_diff = double(ticks(i)) - double(ticks(i-1));

    #handle overflow (wrap-around)
    if abs(raw_diff) > 0.5 * max_tick_value
      if raw_diff > 0
        raw_diff = raw_diff - max_tick_value;
      else
        raw_diff = raw_diff + max_tick_value;
      end
    end

    delta_ticks(i) = raw_diff;
  end
end
