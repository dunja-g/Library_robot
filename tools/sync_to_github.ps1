[CmdletBinding()]
param(
    [switch]$Watch,
    [ValidateRange(2, 300)]
    [int]$IntervalSeconds = 8,
    [string]$Remote = "origin",
    [string]$CommitPrefix = "chore(sync): workspace update"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Invoke-Git {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments)

    & git @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "git $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
    }
}

$repoRoot = (& git -C $PSScriptRoot rev-parse --show-toplevel 2>$null)
if ($LASTEXITCODE -ne 0 -or -not $repoRoot) {
    throw "This script must be run from inside a Git repository."
}

$repoRoot = $repoRoot.Trim()
Set-Location -LiteralPath $repoRoot

$branch = (& git branch --show-current).Trim()
if (-not $branch) {
    throw "No active Git branch was found. Check out a branch before syncing."
}

$remoteUrl = (& git remote get-url $Remote 2>$null)
if ($LASTEXITCODE -ne 0 -or -not $remoteUrl) {
    throw "Git remote '$Remote' is not configured."
}

$gitUserName = (& git config user.name 2>$null)
$gitUserEmail = (& git config user.email 2>$null)
if (-not $gitUserName -or -not $gitUserEmail) {
    throw "Git author identity is missing. Configure 'git config user.name' and 'git config user.email' before syncing."
}

function Sync-Workspace {
    $worktreeStatus = & git status --porcelain
    if (-not $worktreeStatus) {
        Write-Host "[$(Get-Date -Format 'HH:mm:ss')] No changes to sync."
        return
    }

    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Checking remote branch..."
    Invoke-Git fetch $Remote $branch

    $remoteRef = "$Remote/$branch"
    & git show-ref --verify --quiet "refs/remotes/$remoteRef"
    if ($LASTEXITCODE -eq 0) {
        $counts = ((& git rev-list --left-right --count "$remoteRef...HEAD").Trim() -split "\s+")
        $remoteAhead = [int]$counts[0]
        if ($remoteAhead -gt 0) {
            throw "Remote '$remoteRef' has $remoteAhead new commit(s). Pull/rebase before syncing to avoid overwriting teammates' work."
        }
    }

    Invoke-Git add --all

    & git diff --cached --quiet
    if ($LASTEXITCODE -eq 0) {
        Write-Host "[$(Get-Date -Format 'HH:mm:ss')] No staged changes to sync."
        return
    }
    if ($LASTEXITCODE -ne 1) {
        throw "Unable to inspect staged changes."
    }

    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    Invoke-Git commit -m "$CommitPrefix ($timestamp)"
    Invoke-Git push $Remote $branch
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Synced branch '$branch' to $remoteUrl"
}

if (-not $Watch) {
    Sync-Workspace
    exit 0
}

Write-Host "Watching '$repoRoot' every $IntervalSeconds seconds. Press Ctrl+C to stop."
while ($true) {
    Sync-Workspace
    Start-Sleep -Seconds $IntervalSeconds
}
