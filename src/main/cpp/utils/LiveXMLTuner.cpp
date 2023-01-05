/*
have three network table buttons, enable editing, submit changes, reset changes
populate network table with parsed xml values as defaults
create new method thats ran when hitting "submit changes" or "reset changes"
create new variables for each subsystem, parsed value, and actual value which is mutable
when "submit changes" method is ran, and live xml editing is enabled:
    switch to mutable variable, and re-create any objects which need to be created (swerve modules with control data)
when "reset changes" is pressed, run same method if live xml editing is enabled:
    switch to parsed variable, re-create any objects
*/