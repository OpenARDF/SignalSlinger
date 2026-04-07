[CmdletBinding()]
param(
    [string]$SourceBranch = 'Development2',

    [string]$TargetBranch = 'main',

    [switch]$Commit
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

Set-Location -LiteralPath $PSScriptRoot

function Invoke-Git {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments,

        [switch]$AllowFailure
    )

    & git @Arguments
    $exitCode = $LASTEXITCODE

    if((-not $AllowFailure) -and ($exitCode -ne 0))
    {
        throw "git $($Arguments -join ' ') failed with exit code $exitCode."
    }

    return $exitCode
}

function Get-GitOutput {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    $output = & git @Arguments
    if($LASTEXITCODE -ne 0)
    {
        throw "git $($Arguments -join ' ') failed with exit code $LASTEXITCODE."
    }

    return ($output | Out-String).Trim()
}

function Get-GitConfigValue {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    return (Get-GitOutput -Arguments @('config', '--default', '', '--get', $Name))
}

function Test-MergeInProgress {
    & git rev-parse -q --verify MERGE_HEAD *> $null
    return ($LASTEXITCODE -eq 0)
}

function Assert-CleanWorktree {
    $status = Get-GitOutput -Arguments @('status', '--short')
    if($status)
    {
        throw "Working tree must be clean before starting the merge.`n$status"
    }
}

function Ensure-RepoGitConfig {
    $autoCrlf = Get-GitConfigValue -Name 'core.autocrlf'
    if($autoCrlf -ne 'false')
    {
        Invoke-Git -Arguments @('config', 'core.autocrlf', 'false') | Out-Null
        Write-Host 'Set repo-local core.autocrlf=false'
    }

    $safeCrlf = Get-GitConfigValue -Name 'core.safecrlf'
    if($safeCrlf -ne 'true')
    {
        Invoke-Git -Arguments @('config', 'core.safecrlf', 'true') | Out-Null
        Write-Host 'Set repo-local core.safecrlf=true'
    }
}

$mergeInProgress = Test-MergeInProgress
Ensure-RepoGitConfig

if(-not $mergeInProgress)
{
    Assert-CleanWorktree

    $currentBranch = Get-GitOutput -Arguments @('branch', '--show-current')
    if($currentBranch -ne $TargetBranch)
    {
        Invoke-Git -Arguments @('checkout', $TargetBranch) | Out-Null
        Write-Host "Checked out $TargetBranch"
    }

    $mergeExitCode = Invoke-Git -Arguments @('merge', '--no-ff', '--no-commit', $SourceBranch) -AllowFailure
    $mergeInProgress = Test-MergeInProgress

    if(($mergeExitCode -ne 0) -and (-not $mergeInProgress))
    {
        throw "Merge did not start successfully."
    }
}
else
{
    Write-Host 'Merge already in progress. Reusing current merge state.'
}

Invoke-Git -Arguments @('add', '--renormalize', '.') | Out-Null
Invoke-Git -Arguments @('diff', '--cached', '--check') | Out-Null

$conflicts = Get-GitOutput -Arguments @('diff', '--name-only', '--diff-filter=U')
if($conflicts)
{
    Write-Host 'Conflicts remain after renormalization. Resolve these paths, then rerun the script:'
    Write-Host $conflicts
    exit 1
}

Write-Host ''
Write-Host 'Renormalized merge is ready for review.'
Write-Host 'Recommended review commands:'
Write-Host '  git status --short'
Write-Host '  git diff --cached --stat'

if($Commit)
{
    $message = "Merge $SourceBranch into $TargetBranch (renormalized)"
    Invoke-Git -Arguments @('commit', '-m', $message) | Out-Null
    Write-Host "Created merge commit: $message"
}
else
{
    Write-Host 'Create the merge commit after review.'
}
