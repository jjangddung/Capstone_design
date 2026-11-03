function ultraCloseView(f)
if ~isgraphics(f),return;end
if isappdata(f,'UltraLiveSession')
    session=getappdata(f,'UltraLiveSession');rmappdata(f,'UltraLiveSession');
    if isvalid(session),delete(session);end
end
delete(f);
end
