function [legPhases] = gaitGeneration(b)
leg1Pick = b;
leg2Pick = leg1Pick+0.5;
if leg2Pick >= 1
    leg2Pick = leg2Pick - 1;
end
leg3Pick = 2*leg1Pick - 1;
if leg3Pick < 0
    leg3Pick = leg3Pick + 1;
end

leg4Pick = leg3Pick + 0.5;
if leg4Pick >= 1
    leg4Pick = leg4Pick - 1;
end
legPhases = [leg1Pick, leg2Pick, leg3Pick, leg4Pick];
end